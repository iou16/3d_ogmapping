#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H

#include "ros/ros.h"

#include "tf/tf.h"

#include "../utils/stat.h"
#include "../utils/macro_params.h"

namespace ThreeDOGMapping {

	struct MotionModel{
		tf::Pose drawFromMotion(tf::Pose& p, 
													  const tf::Pose& pnew,
                            const tf::Pose& pold,
                            const tf::Transform& base_to_global) const;
		// double srr, str, srt, stt;
	};
};

#endif
