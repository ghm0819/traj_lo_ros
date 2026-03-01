/**
MIT License

Copyright (c) 2023 Xin Zheng <xinzheng@zju.edu.cn>.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
the Software without restriction, including without limitation the rights
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

#ifndef TRAJLO_DATA_LOADER_H
#define TRAJLO_DATA_LOADER_H

#include <tbb/concurrent_queue.h>
#include <atomic>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <trajlo/utils/common_type.h>

#include <filesystem>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

namespace traj {
// since we assume using c++17
namespace fs = std::filesystem;
}  // namespace traj

namespace traj {
class DataLoader {
 public:
  using Ptr = std::shared_ptr<DataLoader>;
  virtual ~DataLoader() = default;
  
  // 移除rosbag处理函数，只保留ROS消息处理函数
  virtual void processStandardPclMsg(const sensor_msgs::PointCloud2::ConstPtr &msg) = 0;
  virtual void processLivoxPclMsg(const livox_ros_driver::CustomMsg::ConstPtr &msg) = 0;

  void bind();
  // laser queue for odometry task
  tbb::concurrent_bounded_queue<Scan::Ptr> *laser_queue = nullptr;

  void start() { is_pub_ = true; }
  void stop() { is_pub_ = false; }

  std::atomic<bool> is_pub_ = false;
};

class DatasetFactory {
 public:
  static DataLoader::Ptr GetDatasetIo(const std::string &dataset_type);
};
}  // namespace traj

#endif  // TRAJLO_DATA_LOADER_H