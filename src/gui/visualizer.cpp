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

#include "trajlo/gui/visualizer.h"

namespace traj {

bool Visualizer::windowInit() {
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit()) return 1;

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only

  // setting forward-compatible core profile context (necessary for macos)
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

  GLFWmonitor* MyMonitor =
      glfwGetPrimaryMonitor();  // The primary monitor.. Later Occulus?..

  const GLFWvidmode* mode = glfwGetVideoMode(MyMonitor);
  width = mode->width;
  height = mode->height;
  // Create window with graphics context
  window = glfwCreateWindow(width, height,
                            "Traj-LO Project. Copyright (c) 2023 Xin Zheng",
                            NULL, NULL);
  if (window == NULL) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // Enable vsync

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize OpenGL context" << std::endl;
    return -1;
  }

  return 1;
}

bool Visualizer::imGuiInit() {
  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();  // implot

  ImGuiIO& io = ImGui::GetIO();
  (void)io;

  ImGui::LoadIniSettingsFromDisk("../data/imgui.ini");
  io.ConfigFlags |=
      ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
  // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable
  // Gamepad Controls
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;  // Enable Docking
                                                     //    io.ConfigFlags |=
                                                     //    ImGuiConfigFlags_ViewportsEnable;
                                                     //    // Enable
  //  Multi-Viewport
  // / Platform Windows
  //   io.ConfigViewportsNoAutoMerge = true;
  // io.ConfigViewportsNoTaskBarIcon = true;

  // Setup Dear ImGui style
  //  ImGui::StyleColorsDark();

  // When viewports are enabled we tweak WindowRounding/WindowBg so platform
  // windows can look identical to regular ones.
  ImGuiStyle& style = ImGui::GetStyle();
  if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
    style.WindowRounding = 0.0f;
    style.Colors[ImGuiCol_WindowBg].w = 1.0f;
  }

  // Setup Platform/Renderer backends
  const char* glsl_version = "#version 150";  // mac need shader version >=140
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  return 1;
}

bool Visualizer::openGLInit() {
  for (int i = 0; i < VA_COUNT; i++) {
    mVAO[i] = 0;
  }
  for (int i = 0; i < VB_COUNT; i++) {
    mVBO[i] = 0;
  }
  for (unsigned int& i : mTexture) {
    i = 0;
  }

  glGenBuffers(VB_COUNT, mVBO);
  glGenVertexArrays(VA_COUNT, mVAO);

  // 轨迹相关
  glBindVertexArray(mVAO[0]);
  glBindBuffer(GL_ARRAY_BUFFER, mVBO[0]);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  // point cloud
  glBindVertexArray(mVAO[1]);
  glBindBuffer(GL_ARRAY_BUFFER, mVBO[1]);
  //  glBufferData(GL_ARRAY_BUFFER, 30000000 * sizeof(float),
  //               NULL, GL_DYNAMIC_DRAW);

  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  // current point cloud
  glBindVertexArray(mVAO[4]);
  glBindBuffer(GL_ARRAY_BUFFER, mVBO[4]);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  // create texture
  glGenTextures(TEXTURE_COUNT, mTexture);
  for (int i = 0; i < TEXTURE_COUNT; i++) {
    glBindTexture(GL_TEXTURE_2D, mTexture[i]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,
                    GL_CLAMP_TO_EDGE);  // This is required on WebGL for non
                                        // power-of-two textures
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,
                    GL_CLAMP_TO_EDGE);  // Same
  }
  glBindTexture(GL_TEXTURE_2D, 0);  // unbind, neet to rebind texture when use

  // framebuffer configuration
  // -------------------------
  glGenFramebuffers(1, &frame_buffer);
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);

  // create a color attachment texture
  glBindTexture(GL_TEXTURE_2D, mTexture[1]);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         mTexture[1], 0);

  glGenFramebuffers(1, &render_buffer);
  glBindFramebuffer(GL_FRAMEBUFFER, render_buffer);

  // create a color attachment texture
  glBindTexture(GL_TEXTURE_2D, mTexture[3]);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         mTexture[3], 0);

  // now that we actually created the framebuffer and added all attachments we
  // want to check if it is actually complete now
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!"
              << std::endl;
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  return true;
}

void Visualizer::mainLoop() {
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // enable docking
    ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());

    { drawPanel(); }
    // Rendering
    ImGui::Render();

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    ImGuiIO& io = ImGui::GetIO();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
      GLFWwindow* backup_current_context = glfwGetCurrentContext();
      ImGui::UpdatePlatformWindows();
      ImGui::RenderPlatformWindowsDefault();
      glfwMakeContextCurrent(backup_current_context);
    }

    glfwSwapBuffers(window);
  }
}
void Visualizer::drawPanel() {
  static float f = 0.0f;
  static int i = 0;
  static int counter = 0;

  ImGui::Begin("Panel");
  ImGui::Checkbox("Odometry", &show_odomerty_window);

  if (show_odomerty_window) {
    static bool odometry_state = false;
    if (odometry_state) {
      ImGui::Text("Traj-LO is working now !!! ");
    } else {
      ImGui::Text("Odometry is not working, you can press the button");
      odometry_state = true;
      show_3D_scene = true;
      show_plot = false;
      odometry->Start();

      if (dataset) {
        dataset->laser_queue = &odometry->laser_data_queue;
      }
    }

    ImGui::Checkbox("3D Scene", &show_3D_scene);
    ImGui::SameLine();
    ImGui::Checkbox("XYZ Plot", &show_plot);

    popVisData();
    if (show_3D_scene) draw3DScene();
    if (show_plot) drawPlot();

    ImGui::Text("Select data source for odometry");
    ImGui::Checkbox("Dataset", &load_dataset);

    if (dataset && load_dataset) {
      if (ImGui::Button("Start")) {
        dataset->start();
      }
      ImGui::SameLine();
      if (ImGui::Button("Stop")) {
        dataset->stop();
      }
    }
  }
  ImGui::NewLine();
  //  ImGui::Dummy({50.0f,50.0f});

  ImGui::NewLine();
  ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
              1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
  ImGui::End();
}

void Visualizer::draw3DScene() {
  ImGui::Begin("Point Cloud 3D Scene");
  ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();  // ImDrawList API uses
                                                   // screen coordinates!
  ImVec2 canvas_sz =
      ImGui::GetContentRegionAvail();  // Resize canvas to what's available
  if (canvas_sz.x < 50.0f) canvas_sz.x = 50.0f;
  if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;
  ImVec2 canvas_p1 =
      ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);
  glBindTexture(GL_TEXTURE_2D, mTexture[1]);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, canvas_sz.x, canvas_sz.y, 0, GL_RGB,
               GL_UNSIGNED_BYTE, NULL);

  //  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glViewport(0, 0, canvas_sz.x, canvas_sz.y);  // main window refresh
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  //  camera->updateTarget(eye, target);
  glm::mat4 pvm = camera->GetPVMMatrix();
  // point cloud
  shader_pointcloud->use();
  shader_pointcloud->setMat4("pvm", pvm);
  glBindVertexArray(mVAO[1]);
  if (buffer_size <= pointcloud.size()) {
    // expand
    glBindBuffer(GL_ARRAY_BUFFER, mVBO[2]);
    glBufferData(GL_ARRAY_BUFFER, buffer_size * sizeof(float), NULL,
                 GL_DYNAMIC_DRAW);
    glBindBuffer(GL_COPY_READ_BUFFER, mVBO[1]);
    glBindBuffer(GL_COPY_WRITE_BUFFER, mVBO[2]);
    glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0,
                        buffer_size * sizeof(float));

    // enlarge vbo and copy back
    buffer_size += buffer_chunk;
    glBindBuffer(GL_ARRAY_BUFFER, mVBO[1]);
    glBufferData(GL_ARRAY_BUFFER, buffer_size * sizeof(float), NULL,
                 GL_DYNAMIC_DRAW);
    glBindBuffer(GL_COPY_READ_BUFFER, mVBO[2]);
    glBindBuffer(GL_COPY_WRITE_BUFFER, mVBO[1]);
    glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0,
                        (buffer_size - buffer_chunk) * sizeof(float));
    std::cout << "enlarge buff\n";
  }
  // subdata
  glBindBuffer(GL_ARRAY_BUFFER, mVBO[1]);
  glBufferSubData(GL_ARRAY_BUFFER, last_points_size * sizeof(float),
                  (pointcloud.size() - last_points_size) * sizeof(float),
                  &pointcloud[0] + last_points_size);
  last_points_size = pointcloud.size();

  glPointSize(1.0f);
  glDrawArrays(GL_POINTS, 0, (int)pointcloud.size() / 4);

  //// current point cloud
  shader_current->use();
  shader_current->setMat4("pvm", pvm);
  glPointSize(3.0f);
  glBindVertexArray(mVAO[4]);
  glBindBuffer(GL_ARRAY_BUFFER, mVBO[4]);
  glBufferData(GL_ARRAY_BUFFER, current_pc.size() * sizeof(float),
               &current_pc[0], GL_STATIC_DRAW);
  glDrawArrays(GL_POINTS, 0, (int)current_pc.size() / 4);

  // draw our trajectory
  shader_trajectory->use();
  shader_trajectory->setMat4("pvm", pvm);
  glBindVertexArray(mVAO[0]);
  glBindBuffer(GL_ARRAY_BUFFER, mVBO[0]);
  glBufferData(GL_ARRAY_BUFFER, trajectory.size() * sizeof(float),
               &trajectory[0], GL_DYNAMIC_DRAW);

  glLineWidth(3.0f);
  glDrawArrays(GL_LINE_STRIP, 0, (int)trajectory.size() / 3);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glDisable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // draw to texture & capture mouse input
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  ImGui::InvisibleButton("trajectory", canvas_sz,
                         ImGuiButtonFlags_MouseButtonLeft |
                             ImGuiButtonFlags_MouseButtonRight |
                             ImGuiButtonFlags_MouseButtonMiddle);
  draw_list->AddImage((void*)(intptr_t)(mTexture[1]), canvas_p0, canvas_p1);

  ImGuiIO& io = ImGui::GetIO();
  const bool is_hovered = ImGui::IsItemHovered();  // Hovered
  const bool is_active = ImGui::IsItemActive();    // Held
  {
    if (is_hovered) {
      float s = io.MouseWheel;
      camera->zoom(s);
    }

    if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
      ImVec2 delta = ImGui::GetMouseDragDelta(2);
      camera->translate(delta.x, delta.y);
    }

    if (is_hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      ImVec2 delta = ImGui::GetMouseDragDelta(0);
      camera->rotate(delta.x, delta.y);
    }

    camera->updateCameraVectors();
  }
  ImGui::End();

  // draw first view sence
  {
    ImGui::Begin("FPV");
    ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();  // ImDrawList API uses
                                                     // screen coordinates!
    ImVec2 canvas_sz =
        ImGui::GetContentRegionAvail();  // Resize canvas to what's available
    if (canvas_sz.x < 50.0f) canvas_sz.x = 500.0f;
    if (canvas_sz.y < 50.0f) canvas_sz.y = 500.0f;
    ImVec2 canvas_p1 =
        ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

    glBindFramebuffer(GL_FRAMEBUFFER, render_buffer);
    glBindTexture(GL_TEXTURE_2D, mTexture[3]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, canvas_sz.x, canvas_sz.y, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, NULL);

    glEnable(GL_DEPTH_TEST);

    glViewport(0, 0, canvas_sz.x, canvas_sz.y);  // 主窗口刷新
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    fpvCam->updateTarget(eye, target);
    glm::mat4 pvm = fpvCam->GetPVMMatrix();
    // point cloud
    shader_pointcloud->use();
    shader_pointcloud->setMat4("pvm", pvm);
    glBindVertexArray(mVAO[1]);

    glPointSize(1.0f);
    glDrawArrays(GL_POINTS, 0, (int)pointcloud.size() / 4);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImGui::InvisibleButton("fpv", canvas_sz,
                           ImGuiButtonFlags_MouseButtonLeft |
                               ImGuiButtonFlags_MouseButtonRight |
                               ImGuiButtonFlags_MouseButtonMiddle);
    draw_list->AddImage((void*)(intptr_t)(mTexture[3]), canvas_p0, canvas_p1);

    ImGui::End();
  }
}

void Visualizer::drawPlot() {
  ImGui::Begin("XYZ Plot");
  static bool paused = false;
  static int last_size = 0;
  if (pData->size - last_size > 0)
    paused = false;
  else
    paused = true;

  if (ImPlot::BeginPlot("##XYZ", ImVec2(-1, -1))) {
    ImPlot::SetupAxisLimits(ImAxis_X1, pData->size - 1000, pData->size + 30,
                            paused ? ImGuiCond_Once : ImGuiCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, -5, 5);

    if (pData->size > 0) {
      ImPlot::PlotLine("X", pData->x.data(), pData->size);
      ImPlot::PlotLine("Y", pData->y.data(), pData->size);
      ImPlot::PlotLine("Z", pData->z.data(), pData->size);

      last_size = pData->size;
    }
    ImPlot::EndPlot();
  }

  ImGui::End();
}

}  // namespace traj
