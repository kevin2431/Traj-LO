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

#ifndef TRAJLO_ROSBAG_LIVOX_H
#define TRAJLO_ROSBAG_LIVOX_H

#include <trajlo/io/data_loader.h>

#define private public
#include <rosbag/bag.h>
#include <rosbag/view.h>
#undef private

#include <livox_ros_driver/CustomMsg.h>

namespace traj {
class RosbagLivox : public DataLoader {
 public:
  RosbagLivox() = default;
  ~RosbagLivox() override = default;

  void pubLivox(const std::string &path, const std::string &point_topic) {
    while (!is_pub_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // read rosbag data a thread
    if (!fs::exists(path))
      std::cerr << "No dataset found in " << path << std::endl;

    bag.reset(new rosbag::Bag);
    bag->open(path, rosbag::bagmode::Read);

    rosbag::View view(*bag);
    // get topics
    std::vector<const rosbag::ConnectionInfo *> connection_infos =
        view.getConnections();

    bool exist_topic = false;
    for (const rosbag::ConnectionInfo *info : connection_infos) {
      if (info->topic == point_topic) {
        exist_topic = true;
        break;
      }
    }
    if (!exist_topic) {
      std::cerr << "LiDAR topic " << point_topic
                << " doesn't exist. Please check the dataset!\n";
      return;
    }

    std::cout << "Livox LiDAR CustomMsg topic " << point_topic << std::endl;
    std::cout << "Start Livox Rosbag data input........\n";

    uint32_t cnt = 0;
    for (const rosbag::MessageInstance &m : view) {
      const std::string &topic = m.getTopic();
      if (point_topic == topic) {
        cnt++;
        Scan::Ptr scan(new Scan);
        livox_ros_driver::CustomMsgConstPtr livox_msg_in =
            m.instantiate<livox_ros_driver::CustomMsg>();

        //                scan->timestamp=livox_msg_in->timebase;
        scan->timestamp = livox_msg_in->header.stamp.toNSec();

        scan->size = livox_msg_in->point_num;
        scan->points.resize(scan->size);

        for (size_t j = 0; j < scan->size; ++j) {
          scan->points[j].x = livox_msg_in->points[j].x;
          scan->points[j].y = livox_msg_in->points[j].y;
          scan->points[j].z = livox_msg_in->points[j].z;
          scan->points[j].intensity = livox_msg_in->points[j].reflectivity;
          scan->points[j].ts =
              (double)(scan->timestamp + livox_msg_in->points[j].offset_time) *
              1e-9;
        }
        if (laser_queue) laser_queue->push(scan);
      }

      if (!is_pub_) break;
    }

    // indicate the end of the bag
    if (laser_queue) laser_queue->push(nullptr);
    std::cout << "Finished Livox dataset input_data thread, totally read "
              << cnt << " LiDAR scans\n";
  }

  void publish(const std::string &path, const std::string &topic) override {
    pubLivox(path, topic);

    return;
  }

 private:
  std::shared_ptr<rosbag::Bag> bag;

  // multi lidar topic
  std::set<std::string> lidar_topic;
  std::map<std::string, std::vector<PointXYZIT>> lidars_cache;
};

}  // namespace traj

#endif  // TRAJLO_ROSBAG_LIVOX_H
