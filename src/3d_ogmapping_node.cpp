#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>

#include "3d_ogmapper/motionmodel.h"
#include "scanmatcher/smmap.h"
#include "scanmatcher/scanmatcher.h"
#include "utils/stat.h"
#include "utils/point.h"
#include "particlefilter/particlefilter.h"

#include <time.h>
#include <assert.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <iostream>
#include <string>

// コマンドラインオプション等の解析
#include <boost/program_options.hpp>


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
    ThreeDOGMappingNode(long unsigned int seed, long unsigned int max_duration_buffer);
	~ThreeDOGMappingNode();

    void init();
    void startReplay(const std::string & bag_fname, std::string point_cloud_topic);

	void pointcloudCallback(const sensor_msgs::PointCloudConstPtr& point_cloud);

  private:
	tf::TransformListener tf_;

    bool first_time;
	
	std::string lidar_frame_id_;
	std::string base_frame_id_;
	std::string odom_frame_id_;
	std::string global_frame_id_;

    ThreeDOGMapping::MotionModel motionmodel_;
    ThreeDOGMapping::ScanMatcher scanmatcher_;

    bool initMapper(const ros::Time& t);
    bool getOdomPose(tf::Pose& pose, const ros::Time& t);

    inline void scanMatch(const pcl::PointCloud<pcl::PointXYZ>& point_cloud);
    inline void normalize();
    inline bool resample(const pcl::PointCloud<pcl::PointXYZ>& point_cloud, int adaptSize = 0); 
    void updateTreeWeights(bool weightsAlreadyNormalized = false);
    inline void resetTree();
    double propagateWeight(ThreeDOGMappingNode::TNode* n, double weight);
    double propagateWeights();
    inline const ParticleVector& getParticles() const {return particles_;};
    int getBestParticleIndex() const;

    // ワンステップ前のオドメトリ座標系での自己位置
	tf::Pose odoPose_;
	// スキャンマッチングの尤度のしきい値
    double minimum_score_;
	// 正規分布の分散
    double sigma_;
	// レンジファインダの尤度場モデルの最近傍の障害物の探索範囲
    int kernelSize_;
	// 各パーティクルに対するスキャンマッチングの探索範囲(的なモノ)
    double lstep_;
    double astep_;
	// 各パーティクルに対するスキャンマッチングの探索回数(的なモノ)
    int iterations_;
    double lsigma_;
	// オドメトリ動作モデルのパラメータ
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double alpha1_;
    double alpha2_;
    double alpha3_;
    double alpha4_;
	// 間引くしきい値＆実際の移動量
    double linerThreshold_, angularThreshold_;
    double linerDistance_,  angularDistance_;
	// 正規化に使う値
    double obsSigmaGain_;
	// リサンプリングのしきい値&neff(neff関連)
    double resampleThreshold_;
    double neff_;
	// パーティクルの数
    int particle_size_;
	// 地図の大きさ
    double xmin_;
    double ymin_;
    double zmin_;
    double xmax_;
    double ymax_;
    double zmax_;
	// voxel一辺の長さ
    double delta_;
	double occThreshold_;

	ros::NodeHandle private_nh_;

	// 擬似乱数の初期値
    unsigned long int seed_;

    ParticleVector particles_;

    std::vector<unsigned int> indexes_;
    std::vector<double> weights_;

    TNode* node;
};

ThreeDOGMappingNode::ThreeDOGMappingNode():
	private_nh_("~"),  tf_(ros::Duration(240))
{
  seed_ = time(NULL);
  init();
}

ThreeDOGMappingNode::ThreeDOGMappingNode(long unsigned int seed, long unsigned int max_duration_buffer):
	private_nh_("~"),  seed_(seed), tf_(ros::Duration(max_duration_buffer))
{
  init();
}

ThreeDOGMappingNode::~ThreeDOGMappingNode(){}

void ThreeDOGMappingNode::init()
{
  first_time = true;

  private_nh_.param("lidar_frame_id", lidar_frame_id_, std::string("hokuyo3d_link"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));	
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));

  private_nh_.param("minimum_score", minimum_score_, 0.0);
  private_nh_.param("sigma", sigma_, 0.05);
  private_nh_.param("kernelSize", kernelSize_, 1);
  private_nh_.param("lstep", lstep_, 0.05);
  private_nh_.param("astep", astep_, 0.05);
  private_nh_.param("iterations", iterations_, 5);
  private_nh_.param("lsigma", lsigma_, 0.075);
  private_nh_.param("srr", srr_, 0.1);
  private_nh_.param("srt", srt_, 0.05);
  private_nh_.param("str", str_, 0.05);
  private_nh_.param("stt", stt_, 0.1);
  private_nh_.param("alpha1", alpha1_, 0.1);
  private_nh_.param("alpha2", alpha2_, 0.1);
  private_nh_.param("alpha3", alpha3_, 0.8);
  private_nh_.param("alpha4", alpha4_, 0.1);

  private_nh_.param("linerThreshold", linerThreshold_, 1.0);
  private_nh_.param("angularThreshold", angularThreshold_, 0.5);
  private_nh_.param("resampleThreshold", resampleThreshold_, 0.5);
  private_nh_.param("particle_size", particle_size_, 30);
  private_nh_.param("xmin", xmin_, -10.0);
  private_nh_.param("ymin", ymin_, -10.0);
  private_nh_.param("zmin", zmin_, -1.0);
  private_nh_.param("xmax", xmax_, 10.0);
  private_nh_.param("ymax", ymax_, 10.0);
  private_nh_.param("zmax", zmax_, 1.0);
  private_nh_.param("delta", delta_, 0.1);
  private_nh_.param("occThreshold", occThreshold_, 0.25);

  obsSigmaGain_=3.0;
}

void ThreeDOGMappingNode::startReplay(const std::string & bag_fname, std::string point_cloud_topic)
{
  double transform_publish_period;
  ros::NodeHandle private_nh_("~");
  
  rosbag::Bag bag;
  bag.open(bag_fname, rosbag::bagmode::Read);
  
  std::vector<std::string> topics;
  topics.push_back(std::string("/tf"));
  topics.push_back(point_cloud_topic);
  rosbag::View viewall(bag, rosbag::TopicQuery(topics));

  std::queue<sensor_msgs::PointCloudConstPtr> s_queue;
  foreach(rosbag::MessageInstance const m, viewall)
  {
    tf::tfMessage::ConstPtr cur_tf = m.instantiate<tf::tfMessage>();
    if (cur_tf != NULL) {
      for (size_t i = 0; i < cur_tf->transforms.size(); ++i)
      {
        geometry_msgs::TransformStamped transformStamped;
        tf::StampedTransform stampedTf;
        transformStamped = cur_tf->transforms[i];
        tf::transformStampedMsgToTF(transformStamped, stampedTf);
        tf_.setTransform(stampedTf);
      }
    }

    sensor_msgs::PointCloudConstPtr s = m.instantiate<sensor_msgs::PointCloud>();
    if (s != NULL) {
      if (!(ros::Time(s->header.stamp)).is_zero())
      {
        s_queue.push(s);
      }
    }

    while (!s_queue.empty())
    {
      this->pointcloudCallback(s_queue.front());
      s_queue.pop();
    }
    
  }

  int best = getBestParticleIndex();
  ROS_INFO("save map");
  pcl::PointCloud<pcl::PointXYZ> pc_msg;
  pc_msg.clear();
  for (int x=0; x < particles_.at(best).map.getMapSizeX(); x++) {
    for (int y=0; y < particles_.at(best).map.getMapSizeY(); y++) {
      for (int z=0; z < particles_.at(best).map.getMapSizeZ(); z++){
        ThreeDOGMapping::IntPoint p(x, y, z);
        double occ=particles_.at(best).map.cell_(p);
        // assert(occ <= 1.0);
        if(occ > occThreshold_) {
          ThreeDOGMapping::Point pc_p = particles_.at(best).map.map2world(x,y,z);
          
          pcl::PointXYZ p_msg(pc_p.x,pc_p.y,pc_p.z);
          pc_msg.push_back(p_msg);
        }
      }
    }
  }
  pcl::io::savePCDFile("3d_map401.pcd", pc_msg);

  bag.close();
}

bool
ThreeDOGMappingNode::initMapper(const ros::Time& t)
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
  motionmodel_.alpha1 = alpha1_;
  motionmodel_.alpha2 = alpha2_;
  motionmodel_.alpha3 = alpha3_;
  motionmodel_.alpha4 = alpha4_;

  ThreeDOGMapping::sampleGaussian(1,seed_);

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

  pose = tf::Pose(tf::createQuaternionFromRPY(0 ,0 ,tf::getYaw(odom_pose.getRotation())), odom_pose.getOrigin());

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
  voxel_cloud = pcl_point_cloud;
  // pcl::VoxelGrid<pcl::PointXYZ> vg;
  // vg.setInputCloud (pcl_point_cloud);
  // vg.setLeafSize (0.1, 0.1, 0.1);
  // vg.filter (*voxel_cloud);


  if (first_time == true) {
    if(!initMapper(point_cloud->header.stamp))
      return;
  }
  
  tf::Pose relPose;
  if (!getOdomPose(relPose, point_cloud->header.stamp))
    return;

  ROS_INFO("sampling");
  motionmodel_.setMotion(relPose, odoPose_);
  for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
     // motionmodel_.drawFromMotion(it->pose_);
     motionmodel_.updateAction(it->pose_);
  }

  tf::Pose move;
  move.setOrigin((relPose.getOrigin() - odoPose_.getOrigin()));
  move.setRotation(relPose.getRotation() * odoPose_.getRotation().inverse());
	double  move_yaw = tf::getYaw(move.getRotation());
  move_yaw = atan2(sin(move_yaw), cos(move_yaw));

  linerDistance_+=std::sqrt(move.getOrigin().distance(tf::Vector3(0,0,0)));
  angularDistance_+=fabs(move_yaw);

  odoPose_ = relPose;

  if (first_time || linerDistance_>linerThreshold_ || angularDistance_ >angularThreshold_) {
    if (first_time) {
      for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
        scanmatcher_.invalidateActiveArea();
        ROS_INFO("computeActiveArea");
        scanmatcher_.computeActiveArea(it->map, it->pose_, *voxel_cloud);
        ROS_INFO("registerScan");
        scanmatcher_.registerScan(it->map, it->pose_, *voxel_cloud);

        TNode* node=new TNode(it->pose_, 0., it->node_, 0);
        it->node_=node;
      }
    } else {
      ROS_INFO("scanMatch");
      scanMatch(*voxel_cloud);

      updateTreeWeights(false);

      ROS_INFO("resample");
      resample(*voxel_cloud);
    }

    updateTreeWeights(false);
    linerDistance_=0;
    angularDistance_=0;

    for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++) it->previousPose_=it->pose_;
  }
  
  if(first_time == true) first_time = false;
  ROS_INFO("END");
}

inline void ThreeDOGMappingNode::scanMatch(const pcl::PointCloud<pcl::PointXYZ>& point_cloud){
  for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
    tf::Pose corrected;
    double score, l, s;
    ROS_INFO("optimize");
    score=scanmatcher_.optimize(corrected, it->map, it->pose_, point_cloud);

    if (score>minimum_score_){
      it->pose_=corrected;
      it->pose_.setRotation(it->pose_.getRotation().normalize());
    }
    
    ROS_INFO("likelihoodAndScore");
    scanmatcher_.likelihoodAndScore(s, l, it->map, it->pose_, point_cloud);
    it->weight_+=l;
    it->weightSum_+=l;

    scanmatcher_.invalidateActiveArea();
    ROS_INFO("computeActiveArea");
    scanmatcher_.computeActiveArea(it->map, it->pose_, point_cloud);
  }
}

inline bool ThreeDOGMappingNode::resample(const pcl::PointCloud<pcl::PointXYZ>& point_cloud, int adaptSize){

  bool hasResampled = false;

  TNodeVector oldGeneration(particles_.size());
#pragma omp parallel for
  for (unsigned int i=0; i<particles_.size(); i++){
		oldGeneration[i] = particles_[i].node_;
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
    scanmatcher_.registerScan(it->map, it->pose_, point_cloud);
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
    scanmatcher_.registerScan(it->map, it->pose_, point_cloud);
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
    namespace po = boost::program_options; 
    po::options_description desc("Options"); 
    desc.add_options() 
    ("help", "Print help messages") 
    ("PointCloud_topic",  po::value<std::string>()->default_value("/hokuyo3d/hokuyo_cloud") ,"topic that contains the PointCcloud in the rosbag")
    ("bag_filename", po::value<std::string>()->required(), "ros bag filename") 
    ("seed", po::value<unsigned long int>()->default_value(0), "seed")
    ("max_duration_buffer", po::value<unsigned long int>()->default_value(999999), "max tf buffer duration") ;
    
    po::variables_map vm; 
    try 
    { 
        po::store(po::parse_command_line(argc, argv, desc),
				  vm); 
        
        if ( vm.count("help")  ) 
        { 
            std::cout << "Basic Command Line Parameter App" << std::endl 
            << desc << std::endl; 
            return 0; 
        } 
        
        po::notify(vm); 
    } 
    catch(po::error& e) 
    { 
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl; 
        std::cerr << desc << std::endl; 
        return -1; 
    } 
    
    std::string bag_fname = vm["bag_filename"].as<std::string>();
    std::string scan_topic = vm["point_cloud_topic"].as<std::string>();
    unsigned long int seed = vm["seed"].as<unsigned long int>();
    unsigned long int max_duration_buffer = vm["max_duration_buffer"].as<unsigned long int>();
    
    ros::init(argc, argv, "threedogmapping");
    ThreeDOGMappingNode tdogmn(seed, max_duration_buffer);
    double start_time = ros::Time::now().toSec();
    tdogmn.startReplay(bag_fname, scan_topic);
    double end_time = ros::Time::now().toSec();
    ROS_INFO("replay stopped.");
    ROS_INFO("time score : %lf", end_time - start_time);

    ros::spin();
    return(0);
}
