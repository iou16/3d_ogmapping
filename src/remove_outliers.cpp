#include <ros/ros.h>

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <iostream>
#include <string>

#include <boost/program_options.hpp>


class RemoveOutliersMap
{
  public:
    RemoveOutliersMap(double max_update_rate) :
      rate_(10)
    {
      rate_ = ros::Rate(max_update_rate);
      ros::NodeHandle nh;
      map_cloud_inliers_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map_cloud_inliers", 0, true);
      map_cloud_outliers_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map_cloud_outliers", 0, true);
    }

    void run(const std::string &pcd_filename, const std::string  &frame_id) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PCDReader reader;
      reader.read<pcl::PointXYZ> (pcd_filename, *map_cloud);

      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (map_cloud);
      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);

      pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
      sor.filter (*map_cloud_inliers);

      sensor_msgs::PointCloud2 map_cloud_inliers_msg;
      pcl::toROSMsg(*map_cloud_inliers,map_cloud_inliers_msg);
      map_cloud_inliers_msg.header.frame_id = frame_id;

      sor.setNegative(true);
      pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>);
      sor.filter (*map_cloud_outliers);

      sensor_msgs::PointCloud2 map_cloud_outliers_msg;
      pcl::toROSMsg(*map_cloud_outliers,map_cloud_outliers_msg);
      map_cloud_outliers_msg.header.frame_id = frame_id;

      while (ros::ok()) {
        map_cloud_inliers_msg.header.stamp = ros::Time::now();
        map_cloud_inliers_pub_.publish(map_cloud_inliers_msg);

        map_cloud_outliers_msg.header.stamp = ros::Time::now();
        map_cloud_outliers_pub_.publish(map_cloud_outliers_msg);

        ros::spinOnce();
        rate_.sleep();
      }
    }

  private:
    ros::Publisher map_cloud_inliers_pub_;
    ros::Publisher map_cloud_outliers_pub_;

    ros::Rate rate_;
};

int main(int argc, char** argv)
{
  namespace po = boost::program_options;
  po::options_description desc("Options");
  desc.add_options()
    ("help", "Print help messages")
    ("pcd_filename", po::value<std::string>()->required(), "pcd filename")
    ("max_update_rate", po::value<double>()->default_value(10.0), "max update rate")
    ("frame_id", po::value<std::string>()->default_value("map"), "target frame_id");
  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help"))
    {
      std::cout << "Basic Command Line Parameter App" << std::endl << desc << std::endl;
      return 0;
    }
    po::notify(vm);
  } catch(po::error& e) {
      std::cerr << "ERROR: " << e.what() << std::endl << std::endl; 
      std::cerr << desc << std::endl; 
      return -1; 
  }

  std::string pcd_filename = vm["pcd_filename"].as<std::string>();
  double max_update_rate = vm["max_update_rate"].as<double>();
  std::string frame_id = vm["frame_id"].as<std::string>();

  ros::init(argc, argv, "removeoutliersmap");

  ros::Time::init();

  RemoveOutliersMap rom(max_update_rate);
  rom.run(pcd_filename, frame_id);
  ros::spin();

  return 0;
}
