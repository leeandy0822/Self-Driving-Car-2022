#include<iostream>
#include<fstream>
#include<limits>
#include<vector>

#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/transform_broadcaster.h>
#include<tf2_eigen/tf2_eigen.h>

#include<Eigen/Dense>

#include<pcl/registration/icp.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl_ros/transforms.h>


class Localizer{
private:

  float mapLeafSize = 1., scanLeafSize = 1.;
  std::vector<float> d_max_list, n_iter_list;

  ros::NodeHandle _nh;
  ros::Subscriber sub_map, sub_points, sub_gps;
  ros::Publisher pub_points, pub_pose;
  tf::TransformBroadcaster br;

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_points;
  pcl::PointXYZ gps_point;
  bool gps_ready = false, map_ready = false, initialied = false;
  Eigen::Matrix4f init_guess;
  int cnt = 0;
  
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;

  std::string result_save_path;
  std::ofstream outfile;
  geometry_msgs::Transform car2Lidar;
  std::string mapFrame, lidarFrame;


public:
  Localizer(ros::NodeHandle nh): map_points(new pcl::PointCloud<pcl::PointXYZI>) {
    std::vector<float> trans, rot;

    _nh = nh;

    _nh.param<std::vector<float>>("baselink2lidar_trans", trans, std::vector<float>());
    _nh.param<std::vector<float>>("baselink2lidar_rot", rot, std::vector<float>());
    _nh.param<std::string>("result_save_path", result_save_path, "result.csv");
    _nh.param<float>("scanLeafSize", scanLeafSize, 1.0);
    _nh.param<float>("mapLeafSize", mapLeafSize, 1.0);
    _nh.param<std::string>("mapFrame", mapFrame, "world");
    _nh.param<std::string>("lidarFrame", lidarFrame, "nuscenes_lidar");

    // ROS_INFO("saving map");

    // ROS_INFO("Origin map cloud size:  %ld", map->points.size());


    ROS_INFO("saving results to %s", result_save_path.c_str());
    outfile.open(result_save_path);
    outfile << "id,x,y,z,yaw,pitch,roll" << std::endl;

    if(trans.size() != 3 | rot.size() != 4){
      ROS_ERROR("transform not set properly");
    }

    car2Lidar.translation.x = trans.at(0);
    car2Lidar.translation.y = trans.at(1);
    car2Lidar.translation.z = trans.at(2);
    car2Lidar.rotation.x = rot.at(0);
    car2Lidar.rotation.y = rot.at(1);
    car2Lidar.rotation.z = rot.at(2);
    car2Lidar.rotation.w = rot.at(3);

    sub_map = _nh.subscribe("/map", 1, &Localizer::map_callback, this);
    sub_points = _nh.subscribe("/lidar_points", 400, &Localizer::pc_callback, this);
    sub_gps = _nh.subscribe("/gps", 1, &Localizer::gps_callback, this);
    pub_points = _nh.advertise<sensor_msgs::PointCloud2>("/transformed_points", 1);
    pub_pose = _nh.advertise<geometry_msgs::PoseStamped>("/lidar_pose", 1);

    init_guess.setIdentity();


    ROS_INFO("%s initialized", ros::this_node::getName().c_str());
  }

  // Gentaly end the node
  ~Localizer(){
    if(outfile.is_open()) outfile.close();
  }

  void map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    ROS_INFO("Got map message");
    pcl::fromROSMsg(*msg, *map_points);
    map_ready = true;
    ROS_INFO("Map point cloud size:  %ld", map_points->points.size());
  }
  
  void pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    ROS_INFO("Got lidar message");
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Matrix4f result;

    while(!(gps_ready & map_ready)){
      ROS_WARN("waiting for map and gps data ...");
      ros::Duration(0.05).sleep();
      ros::spinOnce();
    }

    pcl::fromROSMsg(*msg, *scan_ptr);
    ROS_INFO("point size: %d", scan_ptr->width);
    result = align_map(scan_ptr);

    // publish transformed points
    sensor_msgs::PointCloud2::Ptr out_msg(new sensor_msgs::PointCloud2);
    pcl_ros::transformPointCloud(result, *msg, *out_msg);
    out_msg->header = msg->header;
    out_msg->header.frame_id = mapFrame;
    pub_points.publish(out_msg);

    // broadcast transforms
    tf::Matrix3x3 rot;
    rot.setValue(
      static_cast<double>(result(0, 0)), static_cast<double>(result(0, 1)), static_cast<double>(result(0, 2)), 
      static_cast<double>(result(1, 0)), static_cast<double>(result(1, 1)), static_cast<double>(result(1, 2)),
      static_cast<double>(result(2, 0)), static_cast<double>(result(2, 1)), static_cast<double>(result(2, 2))
    );
    tf::Vector3 trans(result(0, 3), result(1, 3), result(2, 3));
    tf::Transform transform(rot, trans);
    br.sendTransform(tf::StampedTransform(transform.inverse(), msg->header.stamp, lidarFrame, mapFrame));

    // publish lidar pose
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = mapFrame;
    pose.pose.position.x = trans.getX();
    pose.pose.position.y = trans.getY();
    pose.pose.position.z = trans.getZ();
    pose.pose.orientation.x = transform.getRotation().getX();
    pose.pose.orientation.y = transform.getRotation().getY();
    pose.pose.orientation.z = transform.getRotation().getZ();
    pose.pose.orientation.w = transform.getRotation().getW();
    pub_pose.publish(pose);

    Eigen::Affine3d transform_c2l, transform_m2l;
    transform_m2l.matrix() = result.cast<double>();
    transform_c2l = (tf2::transformToEigen(car2Lidar));
    Eigen::Affine3d tf_p = transform_m2l * transform_c2l.inverse();
    geometry_msgs::TransformStamped transform_m2c = tf2::eigenToTransform(tf_p);

    tf::Quaternion q(transform_m2c.transform.rotation.x, transform_m2c.transform.rotation.y, transform_m2c.transform.rotation.z, transform_m2c.transform.rotation.w);
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);
    outfile << ++cnt << "," << tf_p.translation().x() << "," << tf_p.translation().y() << "," << tf_p.translation().z() << "," << yaw << "," << pitch << "," << roll << std::endl;
    std::cout << "\nstored data: " << cnt << "\n"<< std::endl;
  }

  void gps_callback(const geometry_msgs::PointStamped::ConstPtr& msg){
    ROS_INFO("Got GPS message");
    gps_point.x = msg->point.x;
    gps_point.y = msg->point.y;
    gps_point.z = msg->point.z;

    if(!initialied){
    // if(true){
      geometry_msgs::PoseStamped pose;
      pose.header = msg->header;
      pose.pose.position = msg->point;
      pub_pose.publish(pose);
      // ROS_INFO("pub pose");

      tf::Matrix3x3 rot;
      rot.setIdentity();
      tf::Vector3 trans(msg->point.x, msg->point.y, msg->point.z);
      tf::Transform transform(rot, trans);
      br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "world", "nuscenes_lidar"));
    }

    gps_ready = true;
    return;
  }

  Eigen::Matrix4f align_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr scan_points){
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Matrix4f result;

    /* [Part 1] Perform pointcloud preprocessing here e.g. downsampling use setLeafSize(...) ... */
    ROS_INFO("Origin point cloud size:  %ld", scan_points->points.size());

    // set x filter 
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_x;
    voxel_filter_x.setInputCloud(scan_points);
    voxel_filter_x.setLeafSize (0.05f, 0.05f, 0.05f);
    voxel_filter_x.setFilterFieldName("x");
    voxel_filter_x.setFilterLimits(-35, 35);
    voxel_filter_x.filter(*filtered_scan_ptr);

    // set y filter 
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_y;
    voxel_filter_y.setInputCloud(filtered_scan_ptr);
    voxel_filter_y.setLeafSize (0.05f, 0.05f, 0.05f);
    voxel_filter_y.setFilterFieldName("y");
    voxel_filter_y.setFilterLimits(-25, 25);
    voxel_filter_y.filter(*filtered_scan_ptr);

    // set z filter 
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_z;
    voxel_filter_z.setInputCloud(filtered_scan_ptr);
    voxel_filter_z.setLeafSize (0.05f, 0.05f, 0.05f);
    voxel_filter.setFilterFieldName("z");
    voxel_filter_z.setFilterLimits(-1, 5);
    voxel_filter_z.filter(*filtered_scan_ptr);
    ROS_INFO("after filtered: %ld", filtered_scan_ptr->points.size());

    /* Find the initial orientation for fist scan */
    if(!initialied){
      pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> first_icp;
      float yaw, min_yaw, min_score = std::numeric_limits<float>::max();
      Eigen::Matrix4f min_pose(Eigen::Matrix4f::Identity());
	    /* [Part 3] you can perform ICP several times to find a good initial guess */
      // set initial guess
      // localization1: 2.4
      // localization2: ___
      for (yaw = 0; yaw < (M_PI * 2); yaw += 0.6) {
        Eigen::Translation3f init_translation(gps_point.x, gps_point.y, gps_point.z);
        Eigen::AngleAxisf init_rotation_z(yaw, Eigen::Vector3f::UnitZ());
        init_guess = (init_translation * init_rotation_z).matrix();

        icp.setInputSource (filtered_scan_ptr);
        icp.setInputTarget (map_points);
        icp.setMaximumIterations(3000);
        icp.setTransformationEpsilon (1e-14);
        icp.setMaxCorrespondenceDistance (2);
        icp.setEuclideanFitnessEpsilon (0.0001);
        icp.setRANSACOutlierRejectionThreshold (0.02);

        pcl::PointCloud<pcl::PointXYZI> Final;
        icp.align(Final, init_guess);
        std::cout << "Yaw Angle: " << yaw << std::endl;
        std::cout << "First align ICP score: " << icp.getFitnessScore() << std::endl;
        result = icp.getFinalTransformation();
        // std::cout << result << std::endl;
                  
        double score = icp.getFitnessScore(0.5);
        if (score < min_score) {
            min_score = score;
            min_pose = icp.getFinalTransformation();
            ROS_INFO("Update best pose");
        }
      }
      
      init_guess = min_pose;
      initialied = true;
      return init_guess;
    }
	
	/* [Part 2] Perform ICP here or any other scan-matching algorithm */
	/* Refer to https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html#details */
  
  icp.setInputSource (filtered_scan_ptr);
  icp.setInputTarget (map_points);
  icp.setMaximumIterations(3000);
  icp.setTransformationEpsilon (1e-13);
  icp.setMaxCorrespondenceDistance (2);
  icp.setEuclideanFitnessEpsilon (0.0001);
  icp.setRANSACOutlierRejectionThreshold (0.02);

  pcl::PointCloud<pcl::PointXYZI> Final;
  icp.align(Final, init_guess);
  std::cout << "ICP score: " << icp.getFitnessScore() << std::endl;
  result = icp.getFinalTransformation();
  // std::cout << result << std::endl;

	/* Use result as next initial guess */
  init_guess = result;
  return result;

  }
};


int main(int argc, char* argv[]){
  ros::init(argc, argv, "localizer");
  ros::NodeHandle n("~");
  Localizer localizer(n);
  ros::spin();
  return 0;
}
