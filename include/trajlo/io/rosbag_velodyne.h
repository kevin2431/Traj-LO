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

#ifndef TRAJLO_ROSBAG_VELODYNE_H
#define TRAJLO_ROSBAG_VELODYNE_H

#include <trajlo/io/data_loader.h>

#define private public
#include <rosbag/bag.h>
#include <rosbag/view.h>
#undef private

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace traj {
class RosbagVelodyne : public DataLoader {
 public:
  RosbagVelodyne() = default;
  ~RosbagVelodyne() override = default;

  void pubVelodyne(const std::string &path,
                         const std::string &point_topic) {
    while (!is_pub_) {
      // 后续可以用条件变量来控制
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // read rosbag data a thread
    if (!fs::exists(path))
      std::cerr << "No dataset found in " << path << std::endl;

    bag.reset(new rosbag::Bag);
    bag->open(path, rosbag::bagmode::Read);

    rosbag::View view(*bag);

    std::cout << "Velodyne LiDAR CustomMsg topic " << point_topic << std::endl;
    std::cout << "Start Velodyne Rosbag data input........\n";

    uint32_t cnt = 0;
    for (const rosbag::MessageInstance &m : view) {
      const std::string &topic = m.getTopic();
      if (point_topic == topic) {
        cnt++;
        Scan::Ptr scan(new Scan);
        sensor_msgs::PointCloud2Ptr pc_msg =
            m.instantiate<sensor_msgs::PointCloud2>();

        scan->timestamp = pc_msg->header.stamp.toNSec();
        int num = pc_msg->height * pc_msg->width;
        scan->points.resize(num);

        int x_idx = getPointCloud2FieldIndex(*pc_msg, "x");
        int y_idx = getPointCloud2FieldIndex(*pc_msg, "y");
        int z_idx = getPointCloud2FieldIndex(*pc_msg, "z");
        int i_idx = getPointCloud2FieldIndex(*pc_msg, "intensity");

        int ts_idx = getPointCloud2FieldIndex(*pc_msg, "time");

        int x_offset = pc_msg->fields[x_idx].offset;
        int y_offset = pc_msg->fields[y_idx].offset;
        int z_offset = pc_msg->fields[z_idx].offset;
        int i_offset = pc_msg->fields[i_idx].offset;
        int ts_offset = pc_msg->fields[ts_idx].offset;

        int step = pc_msg->point_step;

        double tbase = pc_msg->header.stamp.toSec();
        for (size_t j = 0; j < num; ++j) {
          float x = sensor_msgs::readPointCloud2BufferValue<float>(
              &pc_msg->data[j * step + x_offset],
              pc_msg->fields[x_idx].datatype);
          float y = sensor_msgs::readPointCloud2BufferValue<float>(
              &pc_msg->data[j * step + y_offset],
              pc_msg->fields[y_idx].datatype);
          float z = sensor_msgs::readPointCloud2BufferValue<float>(
              &pc_msg->data[j * step + z_offset],
              pc_msg->fields[z_idx].datatype);
          float intensity = sensor_msgs::readPointCloud2BufferValue<float>(
              &pc_msg->data[j * step + i_offset],
              pc_msg->fields[i_idx].datatype);
          double ts = sensor_msgs::readPointCloud2BufferValue<double>(
              &pc_msg->data[j * step + ts_offset],
              pc_msg->fields[ts_idx].datatype);

          scan->points[j].x = x;
          scan->points[j].y = y;
          scan->points[j].z = z;
          scan->points[j].intensity = intensity;
          scan->points[j].ts = tbase + ts;
        }
        // sort
        std::sort(scan->points.begin(), scan->points.end(),
                  [&](const PointXYZIT &p1, const PointXYZIT &p2) {
                    return p1.ts < p2.ts;
                  });
        scan->size = scan->points.size();
        //        scan->timestamp = (int64_t)(scan->points.front().ts * 1e9);

        if (laser_queue) laser_queue->push(scan);
      }

      if (!is_pub_) break;
    }

    // indicate the end of the bag
    if (laser_queue) laser_queue->push(nullptr);
    std::cout << "Finished Velodyne dataset input_data thread, totally read "
              << cnt << " LiDAR scans\n";
  }

  void publish(const std::string &path, const std::string &topic) override {
    pubVelodyne(path, topic);

    return;
  }

 private:
  std::shared_ptr<rosbag::Bag> bag;

  // multi lidar topic
  std::set<std::string> lidar_topic;
  std::map<std::string, std::vector<PointXYZIT>> lidars_cache;
};

}  // namespace traj

#endif  // TRAJLO_ROSBAG_VELODYNE_H
