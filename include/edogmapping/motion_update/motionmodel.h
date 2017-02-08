#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H

#include "ros/ros.h"

#include "tf/tf.h"

#include "../utils/stat.h"
#include "../utils/macro_params.h"

namespace EDOGMapping {

	struct MotionModel{
      void setMotion(const tf::Pose& pnew, const tf::Pose& pold);
      double normalize(double z);
      double angle_diff(double a, double b);
      tf::Pose updateAction(tf::Pose& p);

      double alpha1, alpha2, alpha3, alpha4;
	  double delta_x, delta_y, delta_yaw;
      double dxy;
	};
};

#endif
