#ifndef SCANMATCHER_H
#define SCANMATCHER_H

#include "ros/ros.h"

#include "tf/tf.h"

#include "smmap.h"
#include <../utils/point.h>
#include <../utils/macro_params.h>
#include <../utils/stat.h>
#include <../utils/gvalues.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <sensor_msgs/PointCloud2.h>

#include <math.h>

namespace EDOGMapping {

class ScanMatcher{
	public:
		ScanMatcher();
		// void set3DLIDARPose(const tf::Pose& lpose);
        void setgenerateMap(bool generateMap);
		void setMatchingParameters
			(double lopt, double aopt, int iterations, double sigma, double likelihoodSigma=1);
        double optimize(tf::Pose& pnew, const ScanMatcherMap& map, const tf::Pose& init, const pcl::PointCloud<pcl::PointXYZ>& point_cloud) const;
		void invalidateActiveArea();
		void computeActiveArea(ScanMatcherMap& map, const tf::Pose& p, const pcl::PointCloud<pcl::PointXYZ>& point_cloud);
		double registerScan(ScanMatcherMap& map, const tf::Pose& p, const pcl::PointCloud<pcl::PointXYZ>& point_cloud);

		inline double score(const ScanMatcherMap& map, const tf::Pose& p, const pcl::PointCloud<pcl::PointXYZ>& point_cloud) const;
        inline void likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const tf::Pose& p, const pcl::PointCloud<pcl::PointXYZ>& point_cloud, ros::Time t) const;
		
		static const double nullLikelihood;
	protected:
		bool m_activeAreaComputed;
		
        // tf::Pose m_3DLIDARPose;
        double m_gaussianSigma;
        double m_likelihoodSigma;
        double m_optAngularDelta, m_optLinearDelta;
        unsigned int m_optRecursiveIterations;
        unsigned int m_likelihoodSkip;
        // bool m_generateMap;
        double m_enlargeStep;
        double m_angularOdometryReliability;
        double m_linearOdometryReliability;

        ros::Publisher test_pub;
};

inline double ScanMatcher::score(const ScanMatcherMap& map, const tf::Pose& p, const pcl::PointCloud<pcl::PointXYZ>& point_cloud) const{
  double s=0;
  tf::Pose lp;
  // tf::Transform base_to_global_ = tf::Transform(p.getRotation());
  // lp.setOrigin(p.getOrigin()+(base_to_global_ * m_3DLIDARPose.getOrigin()));
  // lp.setRotation(p.getRotation() * m_3DLIDARPose.getRotation());
  lp.setOrigin(p.getOrigin());
  lp.setRotation(p.getRotation());
  pcl::PointCloud<pcl::PointXYZ> phit_cloud;
  pcl_ros::transformPointCloud(point_cloud, phit_cloud, lp);

  for (int i=0; i < point_cloud.size(); i++){
    IntPoint iphit=map.world2map(phit_cloud.points.at(i).x,phit_cloud.points.at(i).y);
  	// const PointAccumulator& cell=map.cell_(iphit);
  	const PointAccumulator& cell=map.cell(iphit);
  	double mu=cell.mean()-phit_cloud.points.at(i).z;
  	s+=exp(-1./m_gaussianSigma*mu*mu);
  }

  return s;
}

inline void ScanMatcher::likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const tf::Pose& p, const pcl::PointCloud<pcl::PointXYZ>& point_cloud, ros::Time t) const{
  l=0;
  s=0;
  tf::Pose lp;
  // tf::Transform base_to_global_ = tf::Transform(p.getRotation());
  // lp.setOrigin(p.getOrigin()+(base_to_global_ * m_3DLIDARPose.getOrigin()));
  // lp.setRotation(p.getRotation() * m_3DLIDARPose.getRotation());
  lp.setOrigin(p.getOrigin());
  lp.setRotation(p.getRotation());
  pcl::PointCloud<pcl::PointXYZ> phit_cloud;
  pcl_ros::transformPointCloud(point_cloud, phit_cloud, lp);
  sensor_msgs::PointCloud2 test_cloud2;
  toROSMsg(phit_cloud, test_cloud2);
  test_cloud2.header.stamp = t;
  test_cloud2.header.frame_id = "map";

  // test_pub.publish(test_cloud2);

  for (int i=0; i < point_cloud.size(); i++){
    IntPoint iphit=map.world2map(phit_cloud.points.at(i).x,phit_cloud.points.at(i).y);
  	// const PointAccumulator& cell=map.cell_(iphit);
  	const PointAccumulator& cell=map.cell(iphit);
  	// double mu=cell.gmean()-phit_cloud.points.at(i).z;
  	double mu=cell.mean()-phit_cloud.points.at(i).z;
  	s+=exp(-1./m_gaussianSigma*mu*mu);
    l+=log(phit_cloud.points.at(i).z)+(-1./m_likelihoodSigma)*(mu*mu);
  }
}

};

#endif
