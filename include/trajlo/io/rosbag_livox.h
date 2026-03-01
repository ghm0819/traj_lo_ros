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
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

namespace traj {
class RosbagLivox : public DataLoader {
 public:
  RosbagLivox() = default;
  ~RosbagLivox() override = default;

  void processStandardPclMsg(const sensor_msgs::PointCloud2::ConstPtr &msg) override {
    // Livox不处理标准点云消息，留空实现
  }

  void processLivoxPclMsg(const livox_ros_driver::CustomMsg::ConstPtr &msg) override {
    Scan::Ptr scan(new Scan);
    scan->timestamp = msg->header.stamp.toNSec();

    scan->size = msg->point_num;
    scan->points.resize(scan->size);

    for (size_t j = 0; j < scan->size; ++j) {
      scan->points[j].x = msg->points[j].x;
      scan->points[j].y = msg->points[j].y;
      scan->points[j].z = msg->points[j].z;
      scan->points[j].intensity = msg->points[j].reflectivity;
      scan->points[j].ts =
          (double)(scan->timestamp + msg->points[j].offset_time) * 1e-9;
    }
    
    if (laser_queue) laser_queue->push(scan);
  }

 private:
  // 移除rosbag相关成员变量
};

}  // namespace traj

#endif  // TRAJLO_ROSBAG_LIVOX_H
