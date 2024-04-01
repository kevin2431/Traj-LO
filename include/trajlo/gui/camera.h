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

#ifndef TRAJLO_CAMERA_H
#define TRAJLO_CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <memory>

namespace traj{
class Camera {
 public:
  using Ptr = std::shared_ptr<Camera>;
  Camera(int width = 1280, int height = 720, float fov = 45.0f)
      : width(width), height(height), fov(fov) {
    yaw = 180.0f;
    pitch = 70.0f;
    view_dis = 100.0f;

    camPos = glm::vec3(0.0f, 0.0f, .0f);
    tarPos = glm::vec3(0.0f, 0.0f, 0.0f);

    Up = glm::vec3(0.0f, 0.0f, 1.0f);

    relPos= glm::vec3(-view_dis, 0.0f, .0f);
    camPos=tarPos+relPos;
  }

  void zoom(float s) {
    if (s > 0) view_dis *= 1.2f;
    if (s < 0) view_dis *= 0.8f;
  }

  void rotate(float dx, float dy) {
    if (std::abs(dx) > std::abs(dy)) {
      yaw -= 0.005f * dx;
    } else {
      pitch += 0.005f * dy;
    }
    if (pitch > 89.5f) pitch = 89.5f;
    if (pitch < -89.5f) pitch = -89.5f;
  }

  void translate(float dx, float dy) {
    dx = -dx * 0.00007f * view_dis;
    dy = dy * 0.00007f * view_dis;

    camPos = camPos + camUp * dy + camRight * dx;
    tarPos = tarPos + camUp * dy + camRight * dx;
    //    tarPos+=disPlace;
    //    camPos+=disPlace;
  }

  glm::mat4 GetPVMMatrix() {
    glm::mat4 model = glm::mat4(1.0f);  // Identity
    glm::mat4 view = glm::lookAt(camPos, tarPos, Up);

    glm::mat4 projection = glm::perspective(
        glm::radians(fov), width * 1.0f / height, 0.1f, 10000.0f);
    //    return projection * view*model;

    model[1][1]=-1;
    return projection * model* view;
  }
  void updateCameraVectors() {
    camFront.y =
        std::sin(glm::radians(yaw)) * std::abs(std::cos(glm::radians(pitch)));
    camFront.z = std::sin(glm::radians(pitch));
    camFront.x =
        std::cos(glm::radians(yaw)) * std::abs(std::cos(glm::radians(pitch)));
    camFront = glm::normalize(camFront);

    // for BEV
    //    camFront.y =
    //        std::sin(glm::radians(yaw)) * std::abs(std::sin(glm::radians(pitch)));
    //    camFront.z = std::cos(glm::radians(pitch));
    //    camFront.x =
    //        std::cos(glm::radians(yaw)) * std::abs(std::sin(glm::radians(pitch)));
    //    camFront = glm::normalize(camFront);


    relPos = glm::vec3(camFront.x * view_dis, camFront.y * view_dis,
                       camFront.z * view_dis);

    camPos=tarPos+relPos;

    camRight = glm::cross(Up, camFront);
    camUp = glm::cross(camFront, camRight);
  }

  void updateTarget(const glm::vec3& eye, const glm::vec3& target){
    camPos=eye;
    tarPos=target;
    tarPos.z=eye.z;
  }

 private:
  float fov;
  float yaw;
  float pitch;
  float view_dis;
  glm::vec3 camPos;
  glm::vec3 tarPos;


  glm::vec3 camUp;
  glm::vec3 camFront;
  glm::vec3 camRight;
  glm::vec3 Up;

  glm::vec3 relPos;
  glm::vec3 disPlace;

  int width;
  int height;
};
}

#endif  // TRAJLO_CAMERA_H
