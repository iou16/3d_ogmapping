#include <ros/ros.h>

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "3d_ogmapper/motionmodel.h"
#include "scanmatcher/smmap.h"
#include "scanmatcher/scanmatcher.h"
#include "utils/stat.h"
#include "utils/point.h"
#include "particlefilter/particlefilter.h"

#include <time.h>
#include <assert.h>


class ThreeDOGMappingNode
{
  struct TNode{
    TNode(const tf::Pose& p, double w, TNode* n=0, unsigned int c=0){
      pose_=p;
      weight_=w;
      childs_=c;
      parent_=n;
      // reading_=0;
      gweight_=0;
      if (n){
        n->childs_++;
      }
      flag_=0;
      accWeight_=0;
    }
    ~TNode(){
      if(parent_ && (--parent_->childs_)<=0)
        delete parent_;
      assert(!childs_);
    }
  
    tf::Pose pose_; 
    double weight_;
    double accWeight_;
    double gweight_;
    TNode* parent_;
    // const sensor_msgs::PointCloud* reading_;
    unsigned int childs_;
    mutable unsigned int visitCounter_;
    mutable bool flag_;
  };
  
  typedef std::vector<TNode*> TNodeVector;
  typedef std::deque<TNode*> TNodeDeque;
  
  struct Particle{
    Particle(const ThreeDOGMapping::ScanMatcherMap& m):
      map(m), weight_(0), weightSum_(0), gweight_(0), previousIndex_(0),
      pose_(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0),tf::Point(0,0,0))) {
        node_=0;
      }
  
    inline operator double() const {return weight_;}
    inline operator tf::Pose() const {return pose_;}
  
    inline void setWeight(double w) {weight_=w;}
  
    ThreeDOGMapping::ScanMatcherMap map;
    tf::Pose pose_;
    tf::Pose previousPose_;
    double weight_;
    double weightSum_;
    double gweight_;
    int previousIndex_;
    TNode* node_; 
  };
  
  typedef std::vector<Particle> ParticleVector;


  public:
		ThreeDOGMappingNode();
		~ThreeDOGMappingNode();

    void init();
    void startLiveSlam();
    void publishTransform();

		void pointcloudCallback(const sensor_msgs::PointCloudConstPtr& point_cloud);
    void publishLoop(double transform_publish_period_);

	private:
    ros::Publisher test_pub_1_;
    ros::Publisher test_pub_2_;
    ros::Publisher test_pub_3_;
    ros::Publisher test_pub_4_;

    ros::NodeHandle nh_;
		tf::TransformListener tf_;
    // message_filters::Subscriber<sensor_msgs::PointCloud>* point_cloud_sub_;
    // tf::MessageFilter<sensor_msgs::PointCloud>* point_cloud_filter_;
    ros::Subscriber point_cloud_sub_;
    tf::TransformBroadcaster* tfB_;

    ThreeDOGMapping::MotionModel motionmodel_;
    ThreeDOGMapping::ScanMatcher scanmatcher_;

    bool first_time;

    tf::Transform map_to_odom_;
    boost::mutex map_to_odom_mutex_;

    boost::thread* transform_thread_;

		std::string base_frame_id_;
		std::string odom_frame_id_;
		std::string global_frame_id_;

    bool initMapper(const ros::Time& t, tf::Transform& tf);
    bool getOdomPose(tf::Pose& pose, const ros::Time& t);

    inline void scanMatch(const pcl::PointCloud<pcl::PointXYZ>& point_cloud, tf::Transform& base_to_global );
    inline bool resample(const pcl::PointCloud<pcl::PointXYZ>& point_cloud, const tf::Transform& base_to_global, int adaptSize = 0); 
    inline void normalize();
    inline void resetTree();
    double propagateWeight(ThreeDOGMappingNode::TNode* n, double weight);
    double propagateWeights();
    void updateTreeWeights(bool weightsAlreadyNormalized = false);
    int getBestParticleIndex() const;
    inline const ParticleVector& getParticles() const {return particles_;};

    tf::Pose odoPose_;
    double minimum_score_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linerThreshold_, angularThreshold_;
    double linerDistance_,  angularDistance_;
    double obsSigmaGain_;
    double resampleThreshold_;
    double neff_;
    int particle_size_;
    double xmin_;
    double ymin_;
    double zmin_;
    double xmax_;
    double ymax_;
    double zmax_;
    double delta_;

		ros::NodeHandle private_nh_;

    double transform_publish_period_;
    double tf_delay_;

    ParticleVector particles_;

    std::vector<unsigned int> indexes_;
    std::vector<double> weights_;

    TNode* node;
};

ThreeDOGMappingNode::ThreeDOGMappingNode():
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY(0 ,0 ,0), tf::Point(0, 0, 0))),
	private_nh_("~"), transform_thread_(NULL), tf_(ros::Duration(240))
{
  init();
}

ThreeDOGMappingNode::~ThreeDOGMappingNode()
{
  if(transform_thread_){
    transform_thread_->join();
    delete transform_thread_;
  }
}

void ThreeDOGMappingNode::init()
{
  tfB_ = new tf::TransformBroadcaster();

  first_time = true;

  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
	private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
	private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));

  private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

  private_nh_.param("minimum_score", minimum_score_, 0.0);
  private_nh_.param("sigma", sigma_, 0.05);
  private_nh_.param("kernelSize", kernelSize_, 1);
  private_nh_.param("lstep", lstep_, 0.05);
  private_nh_.param("astep", astep_, 0.05);
  private_nh_.param("iterations", iterations_, 5);
  private_nh_.param("lsigma", lsigma_, 0.075);
  private_nh_.param("srr", srr_, 0.1);
  private_nh_.param("srt", srt_, 0.2);
  private_nh_.param("str", str_, 0.1);
  private_nh_.param("stt", stt_, 0.2);

  private_nh_.param("linerThreshold", linerThreshold_, 1.0);
  private_nh_.param("angularThreshold", angularThreshold_, 0.5);
  private_nh_.param("resampleThreshold", resampleThreshold_, 0.5);
  private_nh_.param("particle_size", particle_size_, 40);
  private_nh_.param("xmin", xmin_, -10.0);
  private_nh_.param("ymin", ymin_, -10.0);
  private_nh_.param("zmin", zmin_, -1.0);
  private_nh_.param("xmax", xmax_, 10.0);
  private_nh_.param("ymax", ymax_, 10.0);
  private_nh_.param("zmax", zmax_, 1.0);
  private_nh_.param("delta", delta_, 0.05);

  private_nh_.param("tf_delay", tf_delay_, transform_publish_period_);

  obsSigmaGain_=3.0;
}

void ThreeDOGMappingNode::startLiveSlam()
{
  point_cloud_sub_ = nh_.subscribe("hokuyo3d/hokuyo_cloud", 0, &ThreeDOGMappingNode::pointcloudCallback, this);
  
  transform_thread_ = new boost::thread(boost::bind(&ThreeDOGMappingNode::publishLoop, this, transform_publish_period_));

  test_pub_1_ = nh_.advertise<geometry_msgs::PoseStamped>("nowpose", 0, true);
  test_pub_2_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 0, true);
  test_pub_3_ = nh_.advertise<sensor_msgs::PointCloud>("pointcloud", 0, true);
  test_pub_4_ = nh_.advertise<geometry_msgs::PoseStamped>("bestpose", 0, true);
}

void
ThreeDOGMappingNode::publishLoop(double transform_publish_period_)
{
  if(transform_publish_period_ == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period_);
  while (ros::ok()){
    publishTransform();
    r.sleep();
  }
}

void
ThreeDOGMappingNode::publishTransform()
{
  map_to_odom_mutex_.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
  tfB_->sendTransform(tf::StampedTransform(map_to_odom_, tf_expiration, global_frame_id_, odom_frame_id_));
  map_to_odom_mutex_.unlock();
}

bool
ThreeDOGMappingNode::initMapper(const ros::Time& t, tf::Transform& tf)
{
  if (!getOdomPose(odoPose_, t))
  {
    odoPose_ = tf::Pose(tf::Transform(tf::createQuaternionFromRPY(0,0,0),tf::Point(0,0,0)));

  }

  particles_.clear();
  TNode* node=new TNode(odoPose_, 0, 0, 0);
  ThreeDOGMapping::ScanMatcherMap lmap(ThreeDOGMapping::Point(xmin_+xmax_, ymin_+ymax_, zmin_+zmax_)*.5, xmax_-xmin_, ymax_-ymin_, zmax_-zmin_, delta_);
  for (unsigned int i=0; i<particle_size_; i++) {
    particles_.push_back(Particle(lmap));
    particles_.back().pose_=odoPose_;
    particles_.back().previousPose_=odoPose_;
    particles_.back().setWeight(0);
    particles_.back().previousIndex_=0;
    particles_.back().node_=node;
  }
  neff_=(double)particle_size_;
  linerDistance_=angularDistance_=0;

  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                           tf::Vector3(0,0,0)), t, "hokuyo3d_link");
  tf::Stamped<tf::Pose> lpose;
  try
  {
    tf_.transformPose(base_frame_id_, ident, lpose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("set3DLIDARPose:%s", e.what());
    return false;
  }
  scanmatcher_.set3DLIDARPose(lpose);
  scanmatcher_.setgenerateMap(false);
  scanmatcher_.setMatchingParameters(kernelSize_, lstep_, astep_, iterations_, sigma_, lsigma_);
  motionmodel_.srr = srr_;
  motionmodel_.srt = srt_;
  motionmodel_.str = str_;
  motionmodel_.stt = stt_;

  tf::Stamped<tf::Quaternion> base_q (tf::createQuaternionFromRPY(0,0,0),
                                      t, base_frame_id_);
  tf::Stamped<tf::Quaternion> base_to_global_q;
   try
   {
     tf_.transformQuaternion(global_frame_id_, base_q, base_to_global_q);
   }
   catch(tf::TransformException e)
   {
    ROS_WARN("base_to_global:%s", e.what());
    return false;
   }
   tf = tf::Transform(base_to_global_q);

  ThreeDOGMapping::sampleGaussian(1,time(NULL));

  return true;
}

bool
ThreeDOGMappingNode::getOdomPose(tf::Pose& pose, const ros::Time& t)
{
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                           tf::Vector3(0,0,0)), t, base_frame_id_);
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_id_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("getOdomPose:%s", e.what());
    return false;
  }

  pose = odom_pose;

  return true;
}

void
ThreeDOGMappingNode::pointcloudCallback(const sensor_msgs::PointCloudConstPtr& point_cloud)
{
  sensor_msgs::PointCloud2 point_cloud2;
  sensor_msgs::convertPointCloudToPointCloud2(*point_cloud, point_cloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(point_cloud2, *pcl_point_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (pcl_point_cloud);
  vg.setLeafSize (0.025, 0.025, 0.025);
  vg.filter (*voxel_cloud);

  static tf::Transform base_to_global;

  if (first_time == true) {
    if(!initMapper(point_cloud->header.stamp, base_to_global))
      return;
  }
  
  tf::Pose relPose;
  if (!getOdomPose(relPose, point_cloud->header.stamp))
    return;

  ROS_INFO("sampling");
  motionmodel_.setMotion(relPose, odoPose_);
  for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
     motionmodel_.drawFromMotion(it->pose_, base_to_global);
  }

  tf::Pose move;
  move.setOrigin((relPose.getOrigin() - odoPose_.getOrigin()));
  move.setRotation(relPose.getRotation() * odoPose_.getRotation().inverse());
	double move_roll, move_pitch, move_yaw;
  tf::Matrix3x3 move_mat =  move.getBasis();
  move_mat.getRPY(move_roll, move_pitch, move_yaw);
  move_roll = atan2(sin(move_roll), cos(move_roll));
  move_pitch = atan2(sin(move_pitch), cos(move_pitch));
  move_yaw = atan2(sin(move_yaw), cos(move_yaw));

  linerDistance_+=std::sqrt(move.getOrigin().distance(tf::Vector3(0,0,0)));
  angularDistance_+=fabs(move_yaw);

  odoPose_ = relPose;

  geometry_msgs::PoseArray cloud_msg;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = global_frame_id_;
  cloud_msg.poses.resize(particles_.size());
  for(int i=0; i<particles_.size(); i++){
    tf::poseTFToMsg(tf::Pose(particles_.at(i).pose_.getRotation(), particles_.at(i).pose_.getOrigin()), cloud_msg.poses[i]);
  }
  test_pub_2_.publish(cloud_msg);

  tf::Stamped<tf::Quaternion> base_q (tf::createQuaternionFromRPY(0,0,0),
                                      point_cloud->header.stamp, base_frame_id_);
  tf::Stamped<tf::Quaternion> base_to_global_q;
  try
  {
    tf_.transformQuaternion(global_frame_id_, base_q, base_to_global_q);
  }
  catch(tf::TransformException e)
  {
   ROS_WARN("base_to_global:%s", e.what());
   return;
  }
  base_to_global = tf::Transform(base_to_global_q);

  if (first_time || linerDistance_>linerThreshold_ || angularDistance_ >angularThreshold_) {
    if (first_time) {
      for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
        scanmatcher_.invalidateActiveArea();
        scanmatcher_.computeActiveArea(it->map, it->pose_, *voxel_cloud, base_to_global);
        ROS_INFO("registerScan");
        scanmatcher_.registerScan(it->map, it->pose_, *voxel_cloud, base_to_global);

        TNode* node=new TNode(it->pose_, 0., it->node_, 0);
        it->node_=node;
      }
    } else {
      ROS_INFO("scanMatch");
      scanMatch(*voxel_cloud, base_to_global);

      geometry_msgs::PoseStamped ps_msg;
      ps_msg.header.stamp = ros::Time::now();
      ps_msg.header.frame_id = global_frame_id_;
      tf::poseTFToMsg(tf::Pose(particles_.at(getBestParticleIndex()).pose_.getRotation(), particles_.at(getBestParticleIndex()).pose_.getOrigin()), ps_msg.pose);
      test_pub_4_.publish(ps_msg);

      updateTreeWeights(false);

      ROS_INFO("resample");
      resample(*voxel_cloud, base_to_global);
      
      geometry_msgs::PoseArray cloud_msg;
      cloud_msg.header.stamp = ros::Time::now();
      cloud_msg.header.frame_id = global_frame_id_;
      cloud_msg.poses.resize(particles_.size());
      for(int i=0; i<particles_.size(); i++){
        tf::poseTFToMsg(tf::Pose(particles_.at(i).pose_.getRotation(), particles_.at(i).pose_.getOrigin()), cloud_msg.poses[i]);
      }
      test_pub_2_.publish(cloud_msg);
    }

    updateTreeWeights(false);
    linerDistance_=0;
    angularDistance_=0;

    for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
	    it->previousPose_=it->pose_;
    }

    int best = getBestParticleIndex();
    tf::Pose mpose = particles_[best].pose_;
    tf::Transform base_to_map = tf::Transform(mpose.getRotation(), mpose.getOrigin()).inverse();
    tf::Transform odom_to_base = tf::Transform(relPose.getRotation(), relPose.getOrigin());

    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_base * base_to_map).inverse();
    map_to_odom_mutex_.unlock();
    
    ROS_INFO("publish map");
    sensor_msgs::PointCloud pc_msg;
    pc_msg.points.clear();
    for (int x=0; x < particles_.at(best).map.getMapSizeX(); x++) {
      for (int y=0; y < particles_.at(best).map.getMapSizeY(); y++) {
        for (int z=0; z < particles_.at(best).map.getMapSizeZ(); z++){
          ThreeDOGMapping::IntPoint p(x, y, z);
          double occ=particles_.at(best).map.cell(p);
          assert(occ <= 1.0);
          if(occ > 0.25) {
            ThreeDOGMapping::Point pc_p = particles_.at(best).map.map2world(x,y,z);
            
            geometry_msgs::Point32 p_msg;
            p_msg.x = pc_p.x;
            p_msg.y = pc_p.y;
            p_msg.z = pc_p.z;
            pc_msg.points.push_back(p_msg);
          }
        }
      }
    }
    pc_msg.header.stamp = ros::Time::now();
    pc_msg.header.frame_id = global_frame_id_;
    test_pub_3_.publish(pc_msg);
  }
  
  if(first_time == true) first_time = false;
  ROS_INFO("END");
}

inline void ThreeDOGMappingNode::scanMatch(const pcl::PointCloud<pcl::PointXYZ>& point_cloud, tf::Transform& base_to_global){
  for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
    tf::Pose corrected;
    double score, l, s;
   // ROS_INFO("optimize");
   // score=scanmatcher_.optimize(corrected, it->map, it->pose_, point_cloud, base_to_global);
   // if (score>minimum_score_){
   //   it->pose_=corrected;
   // }
    scanmatcher_.likelihoodAndScore(s, l, it->map, it->pose_, point_cloud, base_to_global);
    it->weight_+=l;
    it->weightSum_+=l;

    scanmatcher_.invalidateActiveArea();
    scanmatcher_.computeActiveArea(it->map, it->pose_, point_cloud, base_to_global);
  }
}

inline bool ThreeDOGMappingNode::resample(const pcl::PointCloud<pcl::PointXYZ>& point_cloud, const tf::Transform& base_to_global, int adaptSize){

  bool hasResampled = false;

  TNodeVector oldGeneration;
  for (unsigned int i=0; i<particles_.size(); i++){
    oldGeneration.push_back(particles_[i].node_);
  }

  if (neff_<resampleThreshold_*particles_.size()){

  uniform_resampler<double, double> resampler;
  indexes_=resampler.resampleIndexes(weights_, adaptSize);

  ParticleVector temp;
  unsigned int j=0;
  std::vector<unsigned int> deletedParticles;

  for (unsigned int i=0; i<indexes_.size(); i++){
    while(j<indexes_[i]){
	    deletedParticles.push_back(j);
	    j++;
		}
    if (j==indexes_[i])
	    j++;
    Particle & p=particles_[indexes_[i]];
    TNode* node=0;
    TNode* oldNode=oldGeneration[indexes_[i]];
    node=new	TNode(p.pose_, 0, oldNode, 0);
    // node->reading=0;
    
    temp.push_back(p);
    temp.back().node_=node;
    temp.back().previousIndex_=indexes_[i];
  }
  while(j<indexes_.size()){
    deletedParticles.push_back(j);
    j++;
  }
  for (unsigned int i=0; i<deletedParticles.size(); i++){
    delete particles_[deletedParticles[i]].node_;
    particles_[deletedParticles[i]].node_=0;
  }
  
  particles_.clear();
  for (ParticleVector::iterator it=temp.begin(); it!=temp.end(); it++){
    it->setWeight(0);
    scanmatcher_.invalidateActiveArea();
    ROS_INFO("registerScan");
    scanmatcher_.registerScan(it->map, it->pose_, point_cloud, base_to_global);
    particles_.push_back(*it);
  }
  hasResampled = true;
  } else {
  
  int index=0;
  TNodeVector::iterator node_it=oldGeneration.begin();
  for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
    TNode* node=0;
    node=new TNode(it->pose_, 0.0, *node_it, 0);
    
    // node->reading=0;
    it->node_=node;

    scanmatcher_.invalidateActiveArea();
    ROS_INFO("registerScan");
    scanmatcher_.registerScan(it->map, it->pose_, point_cloud, base_to_global);
    it->previousIndex_=index;
    index++;
    node_it++;
    
  }
  }

  return hasResampled;
}

void ThreeDOGMappingNode::updateTreeWeights(bool weightsAlreadyNormalized){
  if (!weightsAlreadyNormalized) {
    ThreeDOGMappingNode::normalize();
  }
  resetTree();
  propagateWeights();
}

inline void ThreeDOGMappingNode::normalize(){
  double gain=1./(obsSigmaGain_*particles_.size());
  double lmax= -std::numeric_limits<double>::max();
  for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
    lmax=it->weight_>lmax?it->weight_:lmax;
  }

  weights_.clear();
  double wcum=0;
  neff_=0;
  for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
    weights_.push_back(exp(gain*(it->weight_-lmax)));
    wcum+=weights_.back();
  }

  neff_=0;
  for (std::vector<double>::iterator it=weights_.begin(); it!=weights_.end(); it++){
    *it=*it/wcum;
    double w=*it;
    neff_+=w*w;
  }
  neff_=1./neff_;
}

inline void ThreeDOGMappingNode::resetTree(){
  for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
    TNode* n=it->node_;
    while (n){
      n->accWeight_=0;
      n->visitCounter_=0;
      n=n->parent_;
    }
  }
}

double ThreeDOGMappingNode::propagateWeight(ThreeDOGMappingNode::TNode* n, double weight){
	if (!n)
		return weight;
	double w=0;
	n->visitCounter_++;
	n->accWeight_+=weight;
	if (n->visitCounter_==n->childs_){
		w=propagateWeight(n->parent_,n->accWeight_);
	}
	assert(n->visitCounter_<=n->childs_);
	return w;
}

double ThreeDOGMappingNode::propagateWeights(){
  double lastNodeWeight=0;
  double aw=0;

  std::vector<double>::iterator w=weights_.begin();
  for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
    double weight=*w;
    aw+=weight;
    TNode * n=it->node_;
    n->accWeight_=weight;
    lastNodeWeight+=propagateWeight(n->parent_,n->accWeight_);
    w++;
  }

  if (fabs(aw-1.0) > 0.0001 || fabs(lastNodeWeight-1.0) > 0.0001) {
	  std::cout << "root->accWeight=" << lastNodeWeight << "sum_leaf_weights=" << aw << std::endl;
    assert(0);
  }
  return lastNodeWeight;
}


int ThreeDOGMappingNode::getBestParticleIndex() const{
  unsigned int bi=0;
  double bw=-std::numeric_limits<double>::max();
  for (unsigned int i=0; i<particles_.size(); i++)
    if (bw<particles_[i].weightSum_){
      bw=particles_[i].weightSum_;
      bi=i;
    }
  return (int) bi;
}


int
main(int argc, char** argv)
{
  ros::init(argc, argv, "threedogmapping");

  ThreeDOGMappingNode tdogmn;
  tdogmn.startLiveSlam();
	ros::spin();

  return 0;
}
