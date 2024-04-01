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
#include <trajlo/io/data_loader.h>
#include <trajlo/io/rosbag_hesai.h>
#include <trajlo/io/rosbag_livox.h>
#include <trajlo/io/rosbag_ouster.h>
#include <trajlo/io/rosbag_velodyne.h>

namespace traj {
DataLoader::Ptr DatasetFactory::GetDatasetIo(const std::string &dataset_type) {
  // rosbag reader according to their type
  if (dataset_type == "bag_livox") {
    return DataLoader::Ptr(new RosbagLivox);
  } else if (dataset_type == "bag_ouster") {
    return DataLoader::Ptr(new RosbagOuster);
  } else if (dataset_type == "bag_hesai") {
    return DataLoader::Ptr(new RosbagHesai);
  } else if (dataset_type == "bag_velodyne") {
    return DataLoader::Ptr(new RosbagVelodyne);
  } else {
    std::cerr << "Dataset type: " << dataset_type << " is not supported\n";
    std::abort();
  }

  return DataLoader::Ptr();
}
}  // namespace traj
