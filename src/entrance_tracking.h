#ifndef ENTRANCE_OBSTACLE_H
#define ENTRANCE_OBSTACLE_H
#include <ros/ros.h>
#include <string>
#include <thread>
//
#include <iostream>
#include <filesystem>
//
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
//
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <trajlo/core/odometry.h>
#include <trajlo/io/data_loader.h>
#include <trajlo/utils/config.h>

using namespace traj;
class EntranceTracking
{
public:
	// 修改构造函数，接受lidar_type和config_file参数
	EntranceTracking(ros::NodeHandle &nh, const std::string& lidar_type, const std::string& config_file);

	~EntranceTracking() = default;

	void Start();

	void Stop();

	void StandardPclCbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
	void LivoxPclCbk(const livox_ros_driver::CustomMsg::ConstPtr &msg);

	EntranceTracking(const EntranceTracking &) = delete;

	EntranceTracking &operator=(const EntranceTracking &) = delete;

	EntranceTracking(EntranceTracking &&) = delete;

	EntranceTracking &operator=(EntranceTracking &&) = delete;

private:

	void MainProcess();

	void VisualizeLoop();

	bool Initializtion();

	std::thread main_processer_;
	std::thread vis_thread_;

	volatile bool start_flag_ = false;

	ros::NodeHandle nh_;

	std::mutex visualization_msgs_lock_;

	ros::Publisher pub_global_pc_;
	ros::Publisher pub_current_pc_;
	ros::Publisher pub_vis_path_;
	ros::Publisher pub_vis_dynamic_;
	tf::TransformBroadcaster tf_broadcaster_;

	// 添加激光雷达类型和配置文件路径成员变量
	std::string lidar_type_;
	std::string config_file_;

	TrajConfig config_;
	TrajLOdometry::Ptr traj_odometry_;
	DataLoader::Ptr data_loader_;

	// path for visualization (from vis data)
	nav_msgs::Path vis_path_msg_;
	nav_msgs::Path vis_dynamic_msg_;
	pcl::PointCloud<pcl::PointXYZI> map_;
	ScanVisData scan_vis_data_;
  	std::vector<std::pair<int64_t, Sophus::SE3d>> trajectory_;
	Eigen::aligned_map<int64_t, PoseStateWithLin<double>> frame_poses_;
};

#endif