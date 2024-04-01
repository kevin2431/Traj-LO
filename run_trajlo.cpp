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

#include <trajlo/core/odometry.h>
#include <trajlo/gui/visualizer.h>
#include <trajlo/io/data_loader.h>
#include <trajlo/utils/config.h>

#include <thread>

int main(int argc, char *argv[]) {
  std::cout << "Hello Traj-LO Project!\n";

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <config_path>" << std::endl;
    return 1;
  }

  std::string config_path = argv[1];

  traj::TrajConfig config;
  config.load(config_path);

  traj::TrajLOdometry::Ptr trajLOdometry(new traj::TrajLOdometry(config));
  traj::DataLoader::Ptr dataLoader = nullptr;
  std::thread t_io;
  dataLoader = traj::DatasetFactory::GetDatasetIo(config.type);
  t_io = std::thread(&traj::DataLoader::publish, dataLoader,
                     config.dataset_path, config.topic);

  traj::Visualizer::Ptr visualizer(
      new traj::Visualizer(trajLOdometry, config, dataLoader));
  std::cout << "Wait data input..............\n";

  { visualizer->mainLoop(); }

  t_io.join();

  return 1;
}