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

class RemoveOutliersMap
{
  public:
    RemoveOutliersMap() :
      rate_(10)
    {
      ros::NodeHandle private_nh("~");
      ros::NodeHandle nh;
      map_cloud_inliers_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map_cloud_inliers", 0, true);
      map_cloud_outliers_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map_cloud_outliers", 0, true);
    }

    void run() {
      while (ros::ok()) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PCDReader reader;
      reader.read<pcl::PointXYZ> ("3d_map.pcd", *map_cloud);
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (map_cloud);
      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);
      pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
      sor.filter (*map_cloud_inliers);
      pcl::toROSMsg(*map_cloud_inliers,map_cloud_inliers_msg);

      sor.setNegative(true);
      pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>);
      sor.filter (*map_cloud_outliers);
      pcl::toROSMsg(*map_cloud_outliers,map_cloud_outliers_msg);
        map_cloud_inliers_msg.header.stamp = ros::Time::now();
        map_cloud_inliers_msg.header.frame_id = "odom";
        map_cloud_inliers_pub_.publish(map_cloud_inliers_msg);

        map_cloud_outliers_msg.header.stamp = ros::Time::now();
        map_cloud_outliers_msg.header.frame_id = "odom";
        map_cloud_outliers_pub_.publish(map_cloud_outliers_msg);

        ros::spinOnce();
        rate_.sleep();
      }
    }

  private:
    sensor_msgs::PointCloud2 map_cloud_inliers_msg;
    sensor_msgs::PointCloud2 map_cloud_outliers_msg;

    ros::Publisher map_cloud_inliers_pub_;
    ros::Publisher map_cloud_outliers_pub_;

    ros::Rate rate_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "removeoutliersmap");

  ros::Time::init();

  RemoveOutliersMap rom;
  rom.run();
  ros::spin();

  return 0;
}
