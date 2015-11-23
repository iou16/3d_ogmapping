#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H

#include "ros/ros.h"

#include "tf/tf.h"

#include "../utils/stat.h"
#include "../utils/macro_params.h"

namespace ThreeDOGMapping {

	struct MotionModel{
    void setMotion(const tf::Pose& pnew, const tf::Pose& pold);
		tf::Pose drawFromMotion(tf::Pose& p);
    double normalize(double z);
    double angle_diff(double a, double b);
    tf::Pose updateAction(tf::Pose& p);
		double srr, str, srt, stt;
		double delta_x, delta_y, /*delta_z, delta_roll, delta_pitch,*/ delta_yaw;
    double dxy;
    double alpha1 = 0.1, alpha2 = 0.1, alpha3 = 0.8, alpha4 = 0.1;
	};
};

#endif
