#include "entrance_tracking.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

// 实现新的构造函数
EntranceTracking::EntranceTracking(ros::NodeHandle &nh, const std::string& lidar_type, const std::string& config_file)
    : nh_(nh), lidar_type_(lidar_type), config_file_(config_file)
{
    static_cast<void>(Initializtion());
}

bool EntranceTracking::Initializtion()
{
    if(!std::filesystem::exists(config_file_) || !std::filesystem::is_regular_file(config_file_)) {
        std::cerr << "检查文件时出错: " << std::endl;
        return false;
    }

    // 初始化TrajLOdometry
	config_.load(config_file_);
    traj_odometry_ = std::make_shared<TrajLOdometry>(config_);

	// 根据激光雷达类型创建对应的数据加载器
    data_loader_ = DatasetFactory::GetDatasetIo(lidar_type_);
    if(data_loader_) {
        data_loader_->laser_queue = &(traj_odometry_->laser_data_queue);
        data_loader_->start();
    } else {
        std::cerr << "Failed to create data loader for lidar type: " << lidar_type_ << std::endl;
		return false;
    }

    // visualization publishers
    pub_global_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("/vis/global_cloud", 1);
    pub_current_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("/vis/current_cloud", 1);
    pub_vis_path_ = nh_.advertise<nav_msgs::Path>("/vis/trajectory", 1);
	pub_vis_dynamic_ = nh_.advertise<nav_msgs::Path>("/vis/dynamic", 1);

    return true;
}

void EntranceTracking::Start()
{
    start_flag_ = true;
    main_processer_ = std::thread(&EntranceTracking::MainProcess, this);
    vis_thread_ = std::thread(&EntranceTracking::VisualizeLoop, this);
}

void EntranceTracking::Stop()
{
    start_flag_ = false;
    if (main_processer_.joinable()) {
        main_processer_.join();
    }
    if (vis_thread_.joinable()) {
        vis_thread_.join();
    }
}

void EntranceTracking::StandardPclCbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    if(data_loader_) {
        data_loader_->processStandardPclMsg(msg);
    }
}

void EntranceTracking::LivoxPclCbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    if(data_loader_) {
        data_loader_->processLivoxPclMsg(msg);
    }
}

void EntranceTracking::MainProcess()
{
    while (start_flag_) {
        if(!traj_odometry_ || !traj_odometry_->hasLidarScan()) {
			usleep(50000);
			continue;
		}
		traj_odometry_->Start(scan_vis_data_);
		{
			std::lock_guard<std::mutex> lock(visualization_msgs_lock_);
			traj_odometry_->CopyVisualizationMsgs(map_, trajectory_, frame_poses_);
		}
    }
}

void EntranceTracking::VisualizeLoop()
{
    sensor_msgs::PointCloud2 msg_current;
	sensor_msgs::PointCloud2 msg_global;
    while (start_flag_) {
		{
			std::lock_guard<std::mutex> lock(visualization_msgs_lock_);
			pcl::PointCloud<pcl::PointXYZI> cloud_current;
    		cloud_current.header.frame_id = "odom";
			for (const auto &p : scan_vis_data_.data) {
            	Eigen::Vector3d pw = scan_vis_data_.T_w * Eigen::Vector3d(p.x, p.y, p.z);
            	pcl::PointXYZI pt;
            	pt.x = static_cast<float>(pw.x());
            	pt.y = static_cast<float>(pw.y());
            	pt.z = static_cast<float>(pw.z());
            	pt.intensity = p.intensity;
            	cloud_current.push_back(pt);
        	}
			pcl::toROSMsg(cloud_current, msg_current);
			pcl::toROSMsg(map_, msg_global);

			nav_msgs::Path vis_path_msg_temp;
			for (const auto& ps_temp : trajectory_) {
				geometry_msgs::PoseStamped ps;
        		ps.header.frame_id = "odom";
        		ps.header.stamp = ros::Time::now();
        		Eigen::Vector3d t = ps_temp.second.translation();
        		Eigen::Quaterniond q = ps_temp.second.unit_quaternion();
        		ps.pose.position.x = t.x();
        		ps.pose.position.y = t.y();
        		ps.pose.position.z = t.z();
        		ps.pose.orientation.x = q.x();
        		ps.pose.orientation.y = q.y();
        		ps.pose.orientation.z = q.z();
        		ps.pose.orientation.w = q.w();
				vis_path_msg_temp.poses.emplace_back(ps);
			}
			vis_path_msg_ = vis_path_msg_temp;

			nav_msgs::Path vis_dynamic_msg_temp;
			for (const auto& ps_temp : frame_poses_) {
				geometry_msgs::PoseStamped ps;
        		ps.header.frame_id = "odom";
        		ps.header.stamp = ros::Time::now();
				auto& pose = ps_temp.second.getPose();
        		Eigen::Vector3d t = pose.translation();
        		Eigen::Quaterniond q = pose.unit_quaternion();
        		ps.pose.position.x = t.x();
        		ps.pose.position.y = t.y();
        		ps.pose.position.z = t.z();
        		ps.pose.orientation.x = q.x();
        		ps.pose.orientation.y = q.y();
        		ps.pose.orientation.z = q.z();
        		ps.pose.orientation.w = q.w();
				vis_dynamic_msg_temp.poses.emplace_back(ps);
			}
			vis_dynamic_msg_ = vis_dynamic_msg_temp;
		}

		if (pub_current_pc_) {
            msg_current.header.stamp = ros::Time::now();
            msg_current.header.frame_id = "odom";
            pub_current_pc_.publish(msg_current);
        }

        if (pub_global_pc_) {
            msg_global.header.stamp = ros::Time::now();
            msg_global.header.frame_id = "odom";
            pub_global_pc_.publish(msg_global);
        }
	
        if (pub_vis_path_) {
			vis_path_msg_.header.stamp = ros::Time::now();
			vis_path_msg_.header.frame_id = "odom";
			pub_vis_path_.publish(vis_path_msg_);
		}
		if (pub_vis_dynamic_) {
			vis_dynamic_msg_.header.stamp = ros::Time::now();
			vis_dynamic_msg_.header.frame_id = "odom";
			pub_vis_dynamic_.publish(vis_dynamic_msg_);
		}
		usleep(40000);
    }
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "tracking");
    ros::NodeHandle nh("~");

    // 获取参数
    std::string scan_topic;
    std::string lidar_type;
    std::string config_file;
    nh.param<std::string>("lidar_topic", scan_topic, "/livox/lidar");
    nh.param<std::string>("lidar_type", lidar_type, "livox");
    nh.param<std::string>("config_file", config_file, "");
    
    // 创建入口跟踪对象，传入激光雷达类型和配置文件
    EntranceTracking enter_node(nh, lidar_type, config_file);

    // 启动主处理线程
    enter_node.Start();

    // 根据激光雷达类型选择订阅相应的回调函数
    ros::Subscriber point_cloud_sub_;
    if (lidar_type == "livox") {
        point_cloud_sub_ = nh.subscribe(
            scan_topic, 1, &EntranceTracking::LivoxPclCbk, &enter_node);
    } else {
        point_cloud_sub_ = nh.subscribe(
            scan_topic, 1, &EntranceTracking::StandardPclCbk, &enter_node);
    }
    
    // 进入ROS事件循环
    ros::spin();
    
    // 停止处理线程
    enter_node.Stop();
}