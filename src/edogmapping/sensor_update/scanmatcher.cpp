#include "scanmatcher.h"

namespace EDOGMapping {


const double ScanMatcher::nullLikelihood=-.5;

ScanMatcher::ScanMatcher()/*: m_3DLIDARPose(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0),tf::Point(0,0,0)))*/{
  m_optRecursiveIterations=3;
  m_activeAreaComputed=false;
  m_enlargeStep=10.;

  m_angularOdometryReliability=0.;
  m_linearOdometryReliability=0.;

  ros::NodeHandle nh;
  test_pub = nh.advertise<sensor_msgs::PointCloud2>("testcloud", 2, true);
}

// void ScanMatcher::set3DLIDARPose(const tf::Pose& lpose){
// 	m_3DLIDARPose=lpose;
// }

// void ScanMatcher::setgenerateMap(bool generateMap){
//   m_generateMap = generateMap;
// }

void ScanMatcher::setMatchingParameters
  (double lopt, double aopt, int iterations, double sigma, double likelihoodSigma){	
	m_optLinearDelta=lopt;
	m_optAngularDelta=aopt;
	m_optRecursiveIterations=iterations;
	m_gaussianSigma=sigma;
	m_likelihoodSigma=likelihoodSigma;
}

void ScanMatcher::invalidateActiveArea(){
	m_activeAreaComputed=false;
}

void ScanMatcher::computeActiveArea(ScanMatcherMap& map, const tf::Pose& p, const pcl::PointCloud<pcl::PointXYZ>& point_cloud){
  if(m_activeAreaComputed)return;
  tf::Pose lp;
  tf::Transform base_to_global_ = tf::Transform(p.getRotation());
  // lp.setOrigin(p.getOrigin()+(base_to_global_ * m_3DLIDARPose.getOrigin()));
  // lp.setRotation(p.getRotation() * m_3DLIDARPose.getRotation());
  lp.setOrigin(p.getOrigin());
  lp.setRotation(p.getRotation());
  // IntPoint p0=map.world2map(Point(lp.getOrigin().x(),lp.getOrigin().y(),lp.getOrigin().z()));
  Point min(map.map2world(0,0));
  Point max(map.map2world(map.getMapSizeX()-1,map.getMapSizeY()-1));
	       
  if (lp.getOrigin().x()<min.x) min.x=lp.getOrigin().x();
  if (lp.getOrigin().y()<min.y) min.y=lp.getOrigin().y();
  if (lp.getOrigin().x()>max.x) max.x=lp.getOrigin().x(); 
  if (lp.getOrigin().y()>max.y) max.y=lp.getOrigin().y(); 

  pcl::PointCloud<pcl::PointXYZ> rotation_point_cloud;
  pcl_ros::transformPointCloud(point_cloud, rotation_point_cloud, lp);

  for (int i=0; i < point_cloud.size(); i++){
  	if (rotation_point_cloud.points.at(i).x<min.x) min.x=rotation_point_cloud.points.at(i).x;
  	if (rotation_point_cloud.points.at(i).y<min.y) min.y=rotation_point_cloud.points.at(i).y;
  	if (rotation_point_cloud.points.at(i).x>max.x) max.x=rotation_point_cloud.points.at(i).x;
  	if (rotation_point_cloud.points.at(i).y>max.y) max.y=rotation_point_cloud.points.at(i).y;
  }

  if(!map.isInside(min) || !map.isInside(max)){
    Point lmin(map.map2world(0,0));
    Point lmax(map.map2world(map.getMapSizeX()-1,map.getMapSizeY()-1));
    min.x=( min.x >= lmin.x )? lmin.x: min.x-m_enlargeStep;
    max.x=( max.x <= lmax.x )? lmax.x: max.x+m_enlargeStep;
    min.y=( min.y >= lmin.y )? lmin.y: min.y-m_enlargeStep;
    max.y=( max.y <= lmax.y )? lmax.y: max.y+m_enlargeStep;
    map.resize(min.x, min.y, max.x, max.y);
  }
	
  HierarchicalArray2D<PointAccumulator>::PointSet activeArea;
  for(int i=0; i < point_cloud.size(); i++){
    IntPoint p1=map.world2map(rotation_point_cloud.points.at(i).x,rotation_point_cloud.points.at(i).y);
    assert(p1.x>=0 && p1.y>=0);
    IntPoint cp=map.storage().patchIndexes(p1);
    assert(cp.x>=0 && cp.y>=0);
    activeArea.insert(cp);
  }
  map.storage().setActiveArea(activeArea, true);
  m_activeAreaComputed=true;
}

double ScanMatcher::registerScan(ScanMatcherMap& map, const tf::Pose& p, const pcl::PointCloud<pcl::PointXYZ>& point_cloud){
  if (!m_activeAreaComputed)
  	computeActiveArea(map, p, point_cloud);
  	
  map.storage().allocActiveArea();
  
  tf::Pose lp;
  // tf::Transform base_to_global_ = tf::Transform(p.getRotation());
  // lp.setOrigin(p.getOrigin()+(base_to_global_ * m_3DLIDARPose.getOrigin()));
  // lp.setRotation(p.getRotation() * m_3DLIDARPose.getRotation());
  lp.setOrigin(p.getOrigin());
  lp.setRotation(p.getRotation());

  pcl::PointCloud<pcl::PointXYZ> rotation_point_cloud;
  pcl_ros::transformPointCloud(point_cloud, rotation_point_cloud, lp);

  double esum=0;
  for (int i=0; i < point_cloud.size(); i++){
  	IntPoint p1=map.world2map(rotation_point_cloud.points.at(i).x,rotation_point_cloud.points.at(i).y);
  	assert(p1.x>=0 && p1.y>=0);
    // for(int x=-1; x<=1; x++){
    //   for(int y=-1; y<=1; y++){
    //     if(x==0 && y==0) {
  	//       map.cell(IntPoint(p1.x+x, p1.y+y)).gupdate(rotation_point_cloud.points.at(i).z/4);
    //     } else if(x==0 || y==0) {
  	//       map.cell(IntPoint(p1.x+x, p1.y+y)).gupdate(rotation_point_cloud.points.at(i).z/8);
    //     } else {
  	//       map.cell(IntPoint(p1.x+x, p1.y+y)).gupdate(rotation_point_cloud.points.at(i).z/16);
    //     }
    //   }
    // }

  	map.cell(p1).update(rotation_point_cloud.points.at(i).z, Point(rotation_point_cloud.points.at(i).x,rotation_point_cloud.points.at(i).y));
  }

  return esum;
}

// double ScanMatcher::optimize(tf::Pose& pnew, const ScanMatcherMap& map, const tf::Pose& init, const pcl::PointCloud<pcl::PointXYZ>& point_cloud) const{
// 	double bestScore=-1;
// 	tf::Pose currentPose=init;
// 	double currentScore=score(map, currentPose, point_cloud);
// 	double adelta=m_optAngularDelta, ldelta=m_optLinearDelta;
// 	unsigned int refinement=0;
// 	enum Move{Front, Back, Left, Right, TurnLeft, TurnRight, Done};
// 	do{
// 		if (bestScore>=currentScore){
// 			refinement++;
// 			adelta*=.5;
// 			ldelta*=.5;
// 		}
// 		bestScore=currentScore;
// 		tf::Pose bestLocalPose=currentPose;
// 
//     double current_yaw = tf::getYaw(currentPose.getRotation());
//     current_yaw = atan2(sin(current_yaw), cos(current_yaw));
// 
//     Pose localPose(currentPose.getOrigin().x(),currentPose.getOrigin().y(),0,0,0,current_yaw);
// 
// 		Move move=Front;
// 		do {
//       current_yaw = tf::getYaw(currentPose.getRotation());
//       current_yaw = atan2(sin(current_yaw), cos(current_yaw));
// 			localPose=Pose(currentPose.getOrigin().x(),currentPose.getOrigin().y(),0,0,0,current_yaw);
// 			switch(move){
// 				case Front:
// 					localPose.x+=ldelta;
// 					move=Back;
// 					break;
// 				case Back:
// 					localPose.x-=ldelta;
// 					move=Left;
// 					break;
// 				case Left:
// 					localPose.y-=ldelta;
// 					move=Right;
// 					break;
// 				case Right:
// 					localPose.y+=ldelta;
// 					move=TurnLeft;
// 					break;
//         case TurnLeft:
//           localPose.yaw+=adelta;
//           move=TurnRight;
//           break;
//         case TurnRight:
//           localPose.yaw-=adelta;
//           move=Done;
//           break;
// 				default:;
// 			}
// 			
// 			double odo_gain=1;
// 			double localScore=odo_gain*score(map, tf::Pose(tf::createQuaternionFromYaw(localPose.yaw),tf::Vector3(localPose.x,localPose.y,0)), point_cloud);
// 			if (localScore>currentScore){
// 				currentScore=localScore;
// 				bestLocalPose=tf::Pose(tf::createQuaternionFromYaw(localPose.yaw),tf::Vector3(localPose.x,localPose.y,0));
// 			}
// 		} while(move!=Done);
// 		currentPose=bestLocalPose;
//     // geometry_msgs::PoseStamped ps_msg;
//     // ps_msg.header.stamp = ros::Time::now();
//     // ps_msg.header.frame_id = "map";
//     // tf::poseTFToMsg(currentPose, ps_msg.pose);
//     // test_pub_2.publish(ps_msg);
// 	}while (currentScore>bestScore || refinement<m_optRecursiveIterations);
// 	pnew=currentPose;
//   // geometry_msgs::PoseStamped ps_msg;
//   // ps_msg.header.stamp = ros::Time::now();
//   // ps_msg.header.frame_id = "map";
//   // tf::poseTFToMsg(pnew, ps_msg.pose);
//   // test_pub_2.publish(ps_msg);
// 	return bestScore;
// }

};
