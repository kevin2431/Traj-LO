/**
MIT License

Copyright (c) 2023 Xin Zheng <xinzheng@zju.edu.cn>.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef TRAJLO_VISUALIZER_H
#define TRAJLO_VISUALIZER_H

// opengl+imgui
#include "glad/glad.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <tbb/concurrent_queue.h>
#include <trajlo/gui/shader.h>
#include <trajlo/utils/common_type.h>
#include <memory>

#include <implot.h>
#include <trajlo/gui/camera.h>

#include <trajlo/core/odometry.h>
#include <trajlo/io/data_loader.h>
#include <trajlo/utils/config.h>

namespace traj {
static void glfw_error_callback(int error, const char* description) {
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

struct plotData {
  using Ptr = std::shared_ptr<plotData>;
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> z;
  int size;
};

class Visualizer {
 public:
  using Ptr = std::shared_ptr<Visualizer>;
  Visualizer(TrajLOdometry::Ptr odometry, TrajConfig& config,
             DataLoader::Ptr dataset, int width = 1280, int height = 720)
      : odometry(odometry),
        config(config),
        dataset(dataset),
        width(width),
        height(height) {
    odometry->vis_data_queue = &vis_data_queue;
    vis_data_queue.set_capacity(30);

    // init
    windowInit();
    imGuiInit();
    openGLInit();

    // current path is ./test
    shader_trajectory.reset(
        new Shader("../include/trajlo/gui/shader/vertexShader.glsl",
                   "../include/trajlo/gui/shader/fragmentShader.glsl"));
    shader_pointcloud.reset(
        new Shader("../include/trajlo/gui/shader/PCVertex.glsl",
                   "../include/trajlo/gui/shader/PCFragment.glsl"));

    shader_current.reset(
        new Shader("../include/trajlo/gui/shader/vertexShader.glsl",
                   "../include/trajlo/gui/shader/current_pc.glsl"));

    pData.reset(new plotData());
    pData->size = 0;

    camera.reset(new Camera(width, height, 45.0f));

    fpvCam.reset(new Camera(width, height, 70.0f));

    lidarPos[0][0] = 1.0f;
    lidarPos[1][1] = 1.0f;
    lidarPos[2][2] = 1.0f;
    lidarPos[3][3] = 1.0f;
  }
  ~Visualizer() {
    glDeleteVertexArrays(VA_COUNT, mVAO);
    glDeleteBuffers(VB_COUNT, mVBO);
    glDeleteTextures(TEXTURE_COUNT, mTexture);
    glDeleteFramebuffers(1, &frame_buffer);
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();

    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    std::cout << "close window and destroy context!!\n";
  }

  inline static void printGlString(const char* name, GLenum s) {
    const char* v = (const char*)glGetString(s);
    printf("GL %s: %s\n", name, v);
  }

  bool windowInit();
  bool imGuiInit();
  bool openGLInit();

  void popVisData() {
    traj::ScanVisData::Ptr data;
    if (vis_data_queue.try_pop(data) && data.get()) {
      std::vector<PointXYZI> h_scan = data->data;

      current_pc.clear();
      for (const auto& p : h_scan) {
        if (p.intensity != 0) {
          Eigen::Vector3f vp(p.x, p.y, p.z);
          vp = data->T_w.cast<float>() * vp;
          pointcloud.push_back(vp(0));
          pointcloud.push_back(vp(1));
          pointcloud.push_back(vp(2));
          //          int I = (int)p.intensity;
          pointcloud.push_back(p.intensity);

          current_pc.push_back(vp(0));
          current_pc.push_back(vp(1));
          current_pc.push_back(vp(2));
          current_pc.push_back(1.0f);
        }
      }

      // to plot data
      pData->x.push_back(data->T_w.translation()[0]);
      pData->y.push_back(data->T_w.translation()[1]);
      pData->z.push_back(data->T_w.translation()[2]);
      pData->size++;

      for (int i = 0; i < 3; i++) {
        trajectory.emplace_back(data->T_w.translation()[i]);
      }

      target.x = data->T_w.translation()[0];
      target.y = data->T_w.translation()[1];
      target.z = data->T_w.translation()[2];

      Eigen::Vector3d temp(-0.5f, .0f, 0.0f);
      temp = data->T_w * temp;
      eye.x = temp[0];
      eye.y = temp[1];
      eye.z = temp[2];

    } else {
    }
  }
  void drawPanel();
  void draw3DScene();
  void drawPlot();

  void mainLoop();

  inline std::string get_date() {
    constexpr int MAX_DATE = 64;
    time_t now;
    char the_date[MAX_DATE];

    the_date[0] = '\0';

    now = time(nullptr);

    if (now != -1) {
      strftime(the_date, MAX_DATE, "%Y_%m_%d_%H_%M_%S", gmtime(&now));
    }

    return std::string(the_date);
  }

  tbb::concurrent_bounded_queue<traj::ScanVisData::Ptr> vis_data_queue;

 private:
  // glfw window
  GLFWwindow* window;
  int width;
  int height;

  // imgui related
  bool show_demo_window = true;
  bool show_another_window = false;
  bool show_record_window = false;
  bool show_odomerty_window = false;

  bool show_3D_scene = true;
  bool show_plot = true;

  bool load_dataset = false;

  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  // opengl shader
  Shader::Ptr shader_trajectory;
  Shader::Ptr shader_pointcloud;
  Shader::Ptr shader_current;

  // opengl buffer
  //  static const int VB_COUNT = 3;
  //  static const int VA_COUNT = 2;
  static const int TEXTURE_COUNT =
      4;  // 0 for range image , 1 for render texture  . when record 0 for image
          // 2 for scan

  static const int VB_COUNT = 5;
  static const int VA_COUNT = 5;

  GLuint mVBO[VB_COUNT]{};
  GLuint mVAO[VA_COUNT]{};
  GLuint mTexture[TEXTURE_COUNT]{};

  GLuint frame_buffer;
  GLuint render_buffer;

  // trajectory
  std::vector<float> trajectory;
  std::vector<float> pointcloud;
  std::vector<float> current_pc;

  size_t buffer_size = 0;
  const size_t buffer_chunk = 100000000;
  //  const size_t buffer_chunk=1200000000;
  size_t last_points_size = 0;

  // plot Data
  plotData::Ptr pData;
  Camera::Ptr camera;
  Camera::Ptr fpvCam;

  //  float3 lidarPos;
  glm::mat4 lidarPos = glm::mat4(.0f);
  glm::vec3 eye = glm::vec3(0.0f);
  glm::vec3 target = glm::vec3(0.0f);

  // data
  TrajLOdometry::Ptr odometry;
  DataLoader::Ptr dataset;
  TrajConfig config;
};
}  // namespace traj

#endif  // TRAJLO_VISUALIZER_H
