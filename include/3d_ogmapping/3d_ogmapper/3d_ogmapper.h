#ifndef THREEDOGMAPPER_H
#define THREEDOGMAPPER_H

#include <climits>
#include <limits>
#include <fstream>
#include <vector>
#include <deque>
// #include <particlefilter/particlefilter.h>
// #include <utils/point.h>
// #include <utils/macro_params.h>
// #include <log/sensorlog.h>
// #include <sensor/sensor_range/rangesensor.h>
// #include <sensor/sensor_range/rangereading.h>
// #include <scanmatcher/scanmatcher.h>
#include "motionmodel.h"


namespace ThreeDOGMapping {
  struct TNode{
    TNode(const tf::Pose& pose, double weight, TNode* parent=0, unsigned int childs=0);
    ~TNode();

    tf::Pose pose; 
    double weight;
    double accWeight;
    double gweight;
    TNode* parent;
    const sensor_msgs::PointCloud* reading;
    unsigned int childs;
    mutable unsigned int visitCounter;
    mutable bool flag;
  };
  
  typedef std::vector<ThreeDOGMapper::TNode*> TNodeVector;
  typedef std::deque<ThreeDOGMapper::TNode*> TNodeDeque;
  
  struct Particle{
    Particle(const ScanMatcherMap& Map);

    inline operator double() const {return weight;}
    inline operator tf::Pose() const {return pose;}

    inline void setWeight(double w) {weight=w;}

    ScanMatcherMap map;
    tf::Pose pose;
    // tf::Pose previousPose;
    double weight;
    double weightSum;
    double gweight;
    int previousIndex;
    TNode* node; 
  };
  
  typedef std::vector<Particle> ParticleVector;
};
