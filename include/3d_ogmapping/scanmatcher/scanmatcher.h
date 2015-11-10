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

#include <sensor_msgs/PointCloud.h>

#include <math.h>

namespace ThreeDOGMapping {

class ScanMatcher{
	public:
		ScanMatcher();
		void set3DLIDARPose(const tf::Pose& lpose);
    void setgenerateMap(bool generateMap);
		void setMatchingParameters
			(int kernsize, double lopt, double aopt, int iterations, double sigma, double likelihoodSigma=1);
    double optimize(tf::Pose pnew, const ScanMatcherMap& map, const tf::Pose& init, const pcl::PointCloud<pcl::PointXYZ>& point_cloud, const tf::Transform& base_to_global) const;
		void invalidateActiveArea();
		void computeActiveArea(ScanMatcherMap& map, const tf::Pose& p, const pcl::PointCloud<pcl::PointXYZ>& point_cloud, const tf::Transform& base_to_global);
		double registerScan(ScanMatcherMap& map, const tf::Pose& p, const pcl::PointCloud<pcl::PointXYZ>& point_cloud, const tf::Transform& base_to_global);

		inline double score(const ScanMatcherMap& map, const tf::Pose& p, const pcl::PointCloud<pcl::PointXYZ>& point_cloud, const tf::Transform& base_to_global) const;
    inline unsigned int likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const tf::Pose& p, const pcl::PointCloud<pcl::PointXYZ>& point_cloud, const tf::Transform& base_to_global) const;
		
		static const double nullLikelihood;
	protected:
		bool m_activeAreaComputed;
		
    tf::Pose m_3DLIDARPose;
    double m_gaussianSigma;
    double m_likelihoodSigma;
    double m_kernelSize;
    double m_optAngularDelta, m_optLinearDelta;
    unsigned int m_optRecursiveIterations;
    unsigned int m_likelihoodSkip;
    bool m_generateMap;
    double m_enlargeStep;
    double m_fullnessThreshold;
    double m_angularOdometryReliability;
    double m_linearOdometryReliability;
    double m_freeCellRatio;

    // ros::Publisher test_pub;
};

inline double ScanMatcher::score(const ScanMatcherMap& map, const tf::Pose& p, const pcl::PointCloud<pcl::PointXYZ>& point_cloud, const tf::Transform& base_to_global) const{
	double s=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_ = point_cloud;
	for (int i=0; i < point_cloud_.size(); i++){
    double r = std::sqrt(std::pow(std::sqrt(std::pow(point_cloud.points.at(i).x,2)+std::pow(point_cloud.points.at(i).y,2)),2)+std::pow(point_cloud.points.at(i).z,2));
    double si, co;
    si = sin(atan2(point_cloud.points.at(i).z, std::sqrt(std::pow(point_cloud.points.at(i).x,2)+std::pow(point_cloud.points.at(i).y,2))));
    co = cos(atan2(point_cloud.points.at(i).z, std::sqrt(std::pow(point_cloud.points.at(i).x,2)+std::pow(point_cloud.points.at(i).y,2))));
		point_cloud_.points.at(i).x = ((r-freeDelta)*co) * cos(atan2(point_cloud.points.at(i).y,point_cloud.points.at(i).x));
		point_cloud_.points.at(i).y = ((r-freeDelta)*co) * sin(atan2(point_cloud.points.at(i).y,point_cloud.points.at(i).x));
    point_cloud_.points.at(i).z = (r-freeDelta)*si; 
  }

  tf::Pose lp;
  lp.setOrigin(p.getOrigin()+(base_to_global * m_3DLIDARPose.getOrigin()));
  lp.setRotation(p.getRotation() * m_3DLIDARPose.getRotation());

  pcl::PointCloud<pcl::PointXYZ> phit_cloud;
  pcl::PointCloud<pcl::PointXYZ> pfree_cloud;
  pcl_ros::transformPointCloud(point_cloud, phit_cloud, lp);
  pcl_ros::transformPointCloud(point_cloud_, pfree_cloud,lp);

  // sensor_msgs::PointCloud pc_msg;
  // pc_msg.points.resize(point_cloud.size());
  // pc_msg.header.stamp = ros::Time::now();    
  // pc_msg.header.frame_id = "map";
  
  for (int i=0; i < point_cloud.size(); i++){
    double r = std::sqrt(std::pow(std::sqrt(std::pow(phit_cloud.points.at(i).x,2)+std::pow(phit_cloud.points.at(i).y,2)),2)+std::pow(phit_cloud.points.at(i).z,2));
	 	if (r>15.0) continue;
    Point phit(phit_cloud.points.at(i).x,phit_cloud.points.at(i).y,phit_cloud.points.at(i).z);
    Point pfree(pfree_cloud.points.at(i).x,pfree_cloud.points.at(i).y,pfree_cloud.points.at(i).z);
		IntPoint iphit=map.world2map(phit_cloud.points.at(i).x,phit_cloud.points.at(i).y,phit_cloud.points.at(i).z);
    pfree=pfree-phit;
		IntPoint ipfree=map.world2map(pfree);
		bool found=false;
		Point bestMu(0.,0.,0.);
		for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
		for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++)
    for (int zz=-m_kernelSize; zz<=m_kernelSize; zz++){
			IntPoint pr=iphit+IntPoint(xx,yy,zz);
			IntPoint pf=pr+ipfree;
				const PointAccumulator& cell=map.cell_(pr);
				const PointAccumulator& fcell=map.cell_(pf);
				if (((double)cell )>m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
					Point mu=phit-cell.mean();
					if (!found){
						bestMu=mu;
						found=true;
					}else
						bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
				}
		}
		if (found)
			s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
    
    // pc_msg.points.at(i).x = pfree_cloud.points.at(i).x;
    // pc_msg.points.at(i).y = pfree_cloud.points.at(i).y;
    // pc_msg.points.at(i).z = pfree_cloud.points.at(i).z;
	}

  // test_pub.publish(pc_msg);
	return s;
}

inline unsigned int ScanMatcher::likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const tf::Pose& p, const pcl::PointCloud<pcl::PointXYZ>& point_cloud, const tf::Transform& base_to_global) const{
	l=0;
	s=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_ = point_cloud;
	for (int i=0; i < point_cloud_.size(); i++){
    double r = std::sqrt(std::pow(std::sqrt(std::pow(point_cloud.points.at(i).x,2)+std::pow(point_cloud.points.at(i).y,2)),2)+std::pow(point_cloud.points.at(i).z,2));
    double si, co;
    si = sin(atan2(point_cloud.points.at(i).z, std::sqrt(std::pow(point_cloud.points.at(i).x,2)+std::pow(point_cloud.points.at(i).y,2))));
    co = cos(atan2(point_cloud.points.at(i).z, std::sqrt(std::pow(point_cloud.points.at(i).x,2)+std::pow(point_cloud.points.at(i).y,2))));
		point_cloud_.points.at(i).x = ((r-freeDelta)*co) * cos(atan2(point_cloud.points.at(i).y,point_cloud.points.at(i).x));
		point_cloud_.points.at(i).y = ((r-freeDelta)*co) * sin(atan2(point_cloud.points.at(i).y,point_cloud.points.at(i).x));
    point_cloud_.points.at(i).z = (r-freeDelta)*si; 
  }

  tf::Pose lp;
  lp.setOrigin(p.getOrigin()+(base_to_global * m_3DLIDARPose.getOrigin()));
  lp.setRotation(p.getRotation() * m_3DLIDARPose.getRotation());

	double noHit=nullLikelihood/(m_likelihoodSigma);
	unsigned int c=0;

  pcl::PointCloud<pcl::PointXYZ> phit_cloud;
  pcl::PointCloud<pcl::PointXYZ> pfree_cloud;
  pcl_ros::transformPointCloud(point_cloud, phit_cloud, lp);
  pcl_ros::transformPointCloud(point_cloud_, pfree_cloud,lp);

  // sensor_msgs::PointCloud pc_msg;
  // pc_msg.points.resize(point_cloud.size());
  // pc_msg.header.stamp = ros::Time::now();    
  // pc_msg.header.frame_id = "map";
  
  for (int i=0; i < point_cloud.size(); i++){
    double r = std::sqrt(std::pow(std::sqrt(std::pow(phit_cloud.points.at(i).x,2)+std::pow(phit_cloud.points.at(i).y,2)),2)+std::pow(phit_cloud.points.at(i).z,2));
	 	if (r>15.0) continue;
    Point phit(phit_cloud.points.at(i).x,phit_cloud.points.at(i).y,phit_cloud.points.at(i).z);
    Point pfree(pfree_cloud.points.at(i).x,pfree_cloud.points.at(i).y,pfree_cloud.points.at(i).z);
		IntPoint iphit=map.world2map(phit_cloud.points.at(i).x,phit_cloud.points.at(i).y,phit_cloud.points.at(i).z);
    pfree=pfree-phit;
		IntPoint ipfree=map.world2map(pfree);
		bool found=false;
		Point bestMu(0.,0.,0.);
		for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
		for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++)
    for (int zz=-m_kernelSize; zz<=m_kernelSize; zz++){
			IntPoint pr=iphit+IntPoint(xx,yy,zz);
			IntPoint pf=pr+ipfree;
				const PointAccumulator& cell=map.cell_(pr);
				const PointAccumulator& fcell=map.cell_(pf);
				if (((double)cell )>m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
					Point mu=phit-cell.mean();
					if (!found){
						bestMu=mu;
						found=true;
					}else
						bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
				}
		}
		if (found){
			s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
			c++;
		}
		double f=(-1./m_likelihoodSigma)*(bestMu*bestMu);
		l+=(found)?f:noHit;
    
    // pc_msg.points.at(i).x = pfree_cloud.points.at(i).x;
    // pc_msg.points.at(i).y = pfree_cloud.points.at(i).y;
    // pc_msg.points.at(i).z = pfree_cloud.points.at(i).z;
	}

  // test_pub.publish(pc_msg);
	return c;
}

};

#endif
