#include "motionmodel.h"
#include <utils/stat.h>
#include <iostream>

#define MotionModelCondtioningLinearCovariance 0.01
#define MotionModelConditoningAngularCovariance 0.001

namespace ThreeDOGMapping {

  void MotionModel::setMotion(const tf::Pose& pnew, const tf::Pose& pold)
    {
      tf::Pose delta_pose;
      tf::Transform odom_to_base(pold.inverse().getRotation());
      delta_pose.setOrigin(odom_to_base * (pnew.getOrigin() - pold.getOrigin()));
      delta_pose.setRotation(pnew.getRotation() * pold.getRotation().inverse());

      delta_x = delta_pose.getOrigin().x();
      delta_y = delta_pose.getOrigin().y();
      delta_yaw = tf::getYaw(delta_pose.getRotation());

			dxy=sqrt(delta_x*delta_x+delta_y*delta_y);
    }

	tf::Pose MotionModel::drawFromMotion(tf::Pose& p) 
		{
			double sxy=0.3*srr;
			delta_x+=sampleGaussian(srr*fabs(delta_x)+str*fabs(delta_yaw)+sxy*fabs(delta_y));
			delta_y+=sampleGaussian(srr*fabs(delta_y)+str*fabs(delta_yaw)+sxy*fabs(delta_x));
			delta_yaw+=sampleGaussian(stt*fabs(delta_yaw)+srt*dxy);
			delta_yaw=fmod(delta_yaw, 2*M_PI);
			if (delta_yaw>M_PI)
				delta_yaw-=2*M_PI;

   		tf::Pose noisy_pose(tf::createQuaternionFromRPY(0,0,delta_yaw),tf::Vector3(delta_x,delta_y,0));
      tf::Transform base_to_global_ = tf::Transform(p.getRotation());
      noisy_pose.setOrigin(base_to_global_ * noisy_pose.getOrigin());
      p.setOrigin(p.getOrigin() + noisy_pose.getOrigin());
      p.setRotation(p.getRotation() * noisy_pose.getRotation());

			return p;
		}
}
