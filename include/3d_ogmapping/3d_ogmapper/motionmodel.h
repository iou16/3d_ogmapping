#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H

#include "ros/ros.h"

#include "tf/tf.h"

#include "../utils/stat.h"
#include "../utils/macro_params.h"

namespace ThreeDOGMapping {

	struct MotionModel{
    void setMotion(const tf::Pose& pnew, const tf::Pose& pold);
		tf::Pose drawFromMotion(tf::Pose& p, tf::Transform base_to_global);
		double srr, str, srt, stt;
		double delta_x, delta_y, /*delta_z, delta_roll, delta_pitch,*/ delta_yaw;
    double dxy;
	};
};

#endif
