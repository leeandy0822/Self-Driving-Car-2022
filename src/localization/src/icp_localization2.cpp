#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "math.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>

#include <string>
#include <fstream>
#include <sstream>




class Localizer
{
private:

  ros::NodeHandle nh; 
  ros::Subscriber sub_map, sub_points, sub_imu;
  ros::Publisher pub_points, pub_pose, pub_map;
  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr load_map;
  sensor_msgs::PointCloud2 map_points;
  Eigen::Matrix4f initial_guess;
  std::ofstream outfile;
  
  int cnt = 0;
  std::string map_path = "/root/catkin_ws/src/data/map/nuscenes_map.pcd";
  std::string csv_path = "/root/catkin_ws/src/localization/result_2.csv";
  std::string target_frame = "car";
  std::string source_frame = "nuscenes_lidar";



public:
  Localizer();
  void lidar_callback(const sensor_msgs::PointCloud2 &msg);
  //void imu_callback(const sensor_msgs::Imu::ConstPtr&msg_imu);
  void imu_callback(const sensor_msgs::Imu &msg_imu);
  Eigen::Matrix4f get_initial_guess();
  Eigen::Matrix4f get_transfrom(std::string link_name);

};

Localizer::Localizer(){

  /// Load Lidar map
  std::cout << "Waiting for initialization..." << std::endl;
  load_map = (new pcl::PointCloud<pcl::PointXYZI>)->makeShared();
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (map_path, *load_map) == -1)
  {
    PCL_ERROR ("Couldn't read the map file. \n");
    exit(0);
  }

  /// Filter map cloud by VoxelGrid
  // pcl::VoxelGrid<pcl::PCLPointCloud2> filted_map;
  // pcl::PCLPointCloud2::Ptr map_cloud2 (new pcl::PCLPointCloud2 ());
  // pcl::toPCLPointCloud2(*load_map, *map_cloud2);
  // filted_map.setInputCloud (map_cloud2);


  // filted_map.setLeafSize (1.0f, 1.0f, 1.0f);
  // filted_map.filter (*map_cloud2);

  // pcl::VoxelGrid<pcl::PointXYZI> sor;
  // sor.setInputCloud (load_map);
  // sor.setLeafSize (0.3f, 0.3f, 0.3f);
  // sor.filter (*load_map);

  pcl::PassThrough<pcl::PointXYZI> passMx;
  passMx.setInputCloud (load_map);
  passMx.setFilterFieldName ("x");
  passMx.setFilterLimits (1600, 2100);
  passMx.filter (*load_map);

  pcl::PassThrough<pcl::PointXYZI> passMy;
  passMy.setInputCloud (load_map);
  passMy.setFilterFieldName ("y");
  passMy.setFilterLimits (750, 950);
  passMy.filter (*load_map);

  pcl::PassThrough<pcl::PointXYZI> passMz;
  passMz.setInputCloud (load_map);
  passMz.setFilterFieldName ("z");
  passMz.setFilterLimits (0, 10);
  passMz.filter (*load_map);



  //pcl::fromPCLPointCloud2(*map_cloud2, *load_map);
  pcl::toROSMsg(*load_map, map_points);
  std::cout << "Now can play the bag file!" << std::endl;


  sub_points = nh.subscribe("lidar_points", 400, &Localizer::lidar_callback, this);
  //sub_imu = nh.subscribe("/imu/data", 400, &Localizer::imu_callback, this);
  pub_points = nh.advertise<sensor_msgs::PointCloud2>("transformed_points", 1);
  pub_pose = nh.advertise<nav_msgs::Odometry>("lidar_pose", 1);
  pub_map = nh.advertise<sensor_msgs::PointCloud2>("map", 1);


  /// Get initial guess
  initial_guess = get_initial_guess();
  std::cout << "Get initial_guess" << std::endl;
  std::cout << initial_guess << std::endl;

  /// Open a csv file
  outfile.open(csv_path, std::ios::app);
  outfile << "id,x,y,z,yaw,pitch,roll" << std::endl;
  outfile.close();

  std::cout << "Initialization done!" << std::endl;
}


Eigen::Matrix4f Localizer::get_initial_guess(){

  Eigen::Matrix4f trans;
  geometry_msgs::PointStampedConstPtr gps;
  sensor_msgs::Imu::ConstPtr imu;

  gps = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/gps", nh);
  imu = ros::topic::waitForMessage<sensor_msgs::Imu>("/imu/data", nh);

  tf::Quaternion q(imu->orientation.x,imu->orientation.y,imu->orientation.z,imu->orientation.w);
  tf::Matrix3x3 m(q);
  double imu_roll, imu_pitch, imu_yaw;
  m.getRPY(imu_roll, imu_pitch, imu_yaw);

  //double yaw = 0.0245 ;//rad
  double yaw = -imu_yaw*M_PI/180;//rad
  std::cout << yaw << std::endl;

  trans << cos(yaw), -sin(yaw), 0,  (*gps).point.x,
           sin(yaw), cos(yaw),  0,  (*gps).point.y,
           0,        0,         1,  (*gps).point.z,
           0,        0,         0,  1;


  return trans;
}



Eigen::Matrix4f Localizer::get_transfrom(std::string link_name){

	tf::StampedTransform transform;
	Eigen::Matrix4f trans;

	try{
    //ros::Duration five_seconds(5.0);
    //listener.waitForTransform("/car", link_name, ros::Time(0), five_seconds);
		listener.lookupTransform(target_frame, link_name, ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return trans;
	}

	Eigen::Quaternionf q(transform.getRotation().getW(), \
		transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
	Eigen::Matrix3f mat = q.toRotationMatrix();
	trans << mat(0,0), mat(0,1), mat(0,2), transform.getOrigin().getX(),
			     mat(1,0), mat(1,1), mat(1,2), transform.getOrigin().getY(),
			     mat(2,0), mat(2,1), mat(2,2), transform.getOrigin().getZ(),
			     0, 0, 0, 1;

	return trans;
}



/***
void Localizer::setGravity(const geometry_msgs::Vector3 &msg)
{
    gravity[0] = msg.x;
    gravity[1] = msg.y;
    gravity[2] = msg.z;
}

void Localizer::calcOrientation(const geometry_msgs::Vector3 &msg)
{
    Eigen::Matrix3d B;
    B << 0,                 -msg.z * deltaT,    msg.y * deltaT,
        msg.z * deltaT,     0,                  -msg.x * deltaT,
        -msg.y * deltaT,    msg.x * deltaT,     0;
    double sigma = std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2)) * deltaT;
    //std::cout << "sigma: " << sigma << std::endl << Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B << std::endl << pose.orien << std::endl;
    pose.orien = pose.orien * (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B - ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);
    //std::cout << "orien" << std::endl;
    //std::cout << pose.orien << std::endl;
}


void Localizer::calcPosition(const geometry_msgs::Vector3 &msg)
{
    Eigen::Vector3d acc_l(msg.x, msg.y, msg.z);
    Eigen::Vector3d acc_g = pose.orien * acc_l;
    //Eigen::Vector3d acc(msg.x - gravity[0], msg.y - gravity[1], msg.z - gravity[2]);
    velocity = velocity + deltaT * (acc_g - gravity);
    //pose.pos = pose.pos + deltaT * velocity;
    pose.pos = deltaT * velocity;

    //std::cout << "pos" << std::endl;
    //std::cout << pose.pos << std::endl;
}


//void Localizer::imu_callback(const sensor_msgs::Imu::ConstPtr&msg_imu)
void Localizer::imu_callback(const sensor_msgs::Imu &msg_imu)
{
  tf::Quaternion q(msg_imu.orientation.x,msg_imu.orientation.y,msg_imu.orientation.z,msg_imu.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);


  // ROS_INFO("Imu Seq: [%d]", msg_imu->header.seq);
  //ROS_INFO("EULER Imu Orientation Roll: [%f], Pitch: [%f], Yaw: [%f]", roll, pitch, yaw);
  //ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg_imu.orientation.x,msg_imu.orientation.y,msg_imu.orientation.z,msg_imu.orientation.w);

  if (firstT) {
      time = msg_imu.header.stamp;
      deltaT = 0;
      setGravity(msg_imu.linear_acceleration);
      firstT = false;
  }
  else {
      deltaT = (msg_imu.header.stamp - time).toSec();
      time = msg_imu.header.stamp;
      calcOrientation(msg_imu.angular_velocity);
      calcPosition(msg_imu.linear_acceleration);
      firstT = true;
  }

}
***/


void Localizer::lidar_callback(const sensor_msgs::PointCloud2 &msg)
{

  pcl::PointCloud<pcl::PointXYZI>::Ptr bag_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(msg, *bag_cloud);

  Eigen::Matrix4f trans = get_transfrom(source_frame);
	transformPointCloud (*bag_cloud, *bag_cloud, trans);


  //std::cout<<"original: "<<bag_cloud->points.size()<<std::endl;
  
  /// Passthrough filter
  pcl::PassThrough<pcl::PointXYZI> pass2;
  pass2.setInputCloud (bag_cloud);
  pass2.setFilterFieldName ("x");
  pass2.setFilterLimits (-35, 35);
  //pass2.setFilterLimits (-40, 40);
  pass2.filter (*bag_cloud);
  //std::cout<<"pass2 filter: "<<bag_cloud->points.size()<<std::endl;

  pcl::PassThrough<pcl::PointXYZI> pass3;
  pass3.setInputCloud (bag_cloud);
  pass3.setFilterFieldName ("y");
  pass3.setFilterLimits (10, 25);
  pass3.filter (*bag_cloud);
  //std::cout<<"pass3 filter: "<<bag_cloud->points.size()<<std::endl;

  pcl::PassThrough<pcl::PointXYZI> pass1;
  pass1.setInputCloud (bag_cloud);
  pass1.setFilterFieldName ("z");
  //pass1.setFilterLimits (1, 50);
  pass1.setFilterLimits (0, 8);
  pass1.filter (*bag_cloud);
  //std::cout<<"pass1 filter: "<<bag_cloud->points.size()<<std::endl;


  /// ICP matching
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(bag_cloud);
  icp.setInputTarget(load_map);

  icp.setMaximumIterations (800);
  icp.setTransformationEpsilon (1e-10);  
  icp.setMaxCorrespondenceDistance (1);  
  icp.setEuclideanFitnessEpsilon (0.0001);  
  icp.setRANSACOutlierRejectionThreshold (0.01);  


  pcl::PointCloud<pcl::PointXYZI> Final;
  icp.align(Final, initial_guess);

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  initial_guess = icp.getFinalTransformation();



  tf::Matrix3x3 tf3d;
  tf3d.setValue((initial_guess(0,0)), (initial_guess(0,1)), (initial_guess(0,2)),
                (initial_guess(1,0)), (initial_guess(1,1)), (initial_guess(1,2)),
                (initial_guess(2,0)), (initial_guess(2,1)), (initial_guess(2,2)));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(initial_guess(0,3),initial_guess(1,3),initial_guess(2,3)));
  transform.setRotation(tfqt);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),target_frame,"world"));
  

  /// Show the map
  map_points.header.frame_id = "world";
  map_points.header.stamp = ros::Time::now();
  pub_map.publish(map_points);


  /// Publish the pointcloud after ICP
  sensor_msgs::PointCloud2 final_cloud;
  pcl::toROSMsg(Final, final_cloud);
  final_cloud.header=msg.header;
  final_cloud.header.frame_id = "world";
  pub_points.publish(final_cloud);


  /// Publish the result as nav_msgs/Odometry.msg
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.child_frame_id = target_frame;
  odom.pose.pose.position.x = initial_guess(0,3);
  odom.pose.pose.position.y = initial_guess(1,3);
  odom.pose.pose.position.z = initial_guess(2,3);
  tf2::Matrix3x3 m;
  m.setValue(initial_guess(0,0) ,initial_guess(0,1) ,initial_guess(0,2) ,
             initial_guess(1,0) ,initial_guess(1,1) ,initial_guess(1,2) ,
             initial_guess(2,0) ,initial_guess(2,1) ,initial_guess(2,2));
  tf2::Quaternion tfq2;
  m.getRotation(tfq2);
  odom.pose.pose.orientation.x = tfq2[0];
  odom.pose.pose.orientation.y = tfq2[1];
  odom.pose.pose.orientation.z = tfq2[2];
  odom.pose.pose.orientation.w = tfq2[3];
  pub_pose.publish(odom);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);


  /// Save the result to csv file
  std::cout<<"cnt="<<cnt<<std::endl;
  outfile.open(csv_path, std::ios::app);
  outfile << ++cnt <<','<< odom.pose.pose.position.x << ',' << odom.pose.pose.position.y << ',' << odom.pose.pose.position.z <<','<< yaw << ',' << pitch << ',' << roll <<std::endl;
  outfile.close();
}



int main (int argc, char** argv)
{
  ros::init(argc, argv, "Localizer");
  Localizer icp;
  ros::spin();
  return 0;

}
