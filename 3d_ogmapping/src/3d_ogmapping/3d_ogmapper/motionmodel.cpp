#include "motionmodel.h"
#include <utils/stat.h>
#include <iostream>

#define MotionModelCondtioningLinearCovariance 0.01
#define MotionModelConditoningAngularCovariance 0.001

namespace ThreeDOGMapping {

	tf::Pose
		MotionModel::drawFromMotion(tf::Pose& p,
                                const tf::Pose& pnew,
                                const tf::Pose& pold,
                                const tf::Transform& base_to_global) 
		const{
			// double 0.03=0.3*srr;
      tf::Pose delta_pose;
      tf::Transform odom_to_base_(pold.inverse().getRotation());
      delta_pose.setOrigin(odom_to_base_ * (pnew.getOrigin() - pold.getOrigin()));
      delta_pose.setRotation(pnew.getRotation() * pold.getRotation().inverse());

			double delta_x, delta_y, delta_z, delta_roll, delta_pitch, delta_yaw;
      delta_x = delta_pose.getOrigin().x();
      delta_y = delta_pose.getOrigin().y();
      delta_z = delta_pose.getOrigin().z();
      tf::Matrix3x3 delta_mat =  delta_pose.getBasis();
      delta_mat.getRPY(delta_roll, delta_pitch, delta_yaw);
      delta_roll = atan2(sin(delta_roll), cos(delta_roll));
      delta_pitch = atan2(sin(delta_pitch), cos(delta_pitch));
      delta_yaw = atan2(sin(delta_yaw), cos(delta_yaw));

			double dxy=sqrt(delta_x*delta_x+delta_y*delta_y);
			double dxyz=sqrt(dxy*dxy+delta_z*delta_z);
			delta_x+=sampleGaussian(0.1*fabs(delta_x)+0.03*fabs(delta_y)+0.1*fabs(delta_yaw)+0.01*fabs(delta_z)+0.01*fabs(delta_pitch));
			delta_y+=sampleGaussian(0.1*fabs(delta_y)+0.03*fabs(delta_x)+0.1*fabs(delta_yaw)+0.01*fabs(delta_z)+0.01*fabs(delta_roll));
			delta_z+=sampleGaussian(0.1*fabs(delta_z)+fabs(delta_x)*0.01+fabs(delta_y)*0.01+fabs(delta_roll)*0.01+fabs(delta_pitch)*0.1);
			delta_roll+=sampleGaussian(0.2*fabs(delta_roll)+0.01*fabs(delta_pitch)+0.01*fabs(delta_yaw)+0.1*sqrt(dxy*dxy+delta_z*delta_z));
			delta_pitch+=sampleGaussian(0.01*fabs(delta_roll)+0.2*fabs(delta_pitch)+0.01*fabs(delta_yaw)+0.1*sqrt(dxy*dxy+delta_z*delta_z));
			delta_yaw+=sampleGaussian(0.01*fabs(delta_roll)+0.01*fabs(delta_pitch)+0.2*fabs(delta_yaw)+0.1*sqrt(dxy*dxy+delta_z*delta_z));
			delta_roll=fmod(delta_roll, 2*M_PI);
			delta_pitch=fmod(delta_pitch, 2*M_PI);
			delta_yaw=fmod(delta_yaw, 2*M_PI);
			if (delta_roll>M_PI)
				delta_roll-=2*M_PI;
			if (delta_pitch>M_PI)
				delta_pitch-=2*M_PI;
			if (delta_yaw>M_PI)
				delta_yaw-=2*M_PI;

   		tf::Pose noisy_pose(tf::createQuaternionFromRPY(delta_roll,delta_pitch,delta_yaw),tf::Vector3(delta_x,delta_y,delta_z));
      tf::Transform base_to_global_(p.inverse().getRotation());
      // noisy_pose.setOrigin(base_to_global * noisy_pose.getOrigin());
      noisy_pose.setOrigin(base_to_global_ * noisy_pose.getOrigin());
      // noisy_pose.setRotation(noisy_pose.getRotation());
      p.setOrigin(p.getOrigin() + noisy_pose.getOrigin());
      p.setRotation(p.getRotation() * noisy_pose.getRotation());

			return p;
		}
}
