#include "motionmodel.h"
#include <utils/stat.h>
#include <iostream>

#define MotionModelCondtioningLinearCovariance 0.01
#define MotionModelConditoningAngularCovariance 0.001

namespace EDOGMapping {

  void MotionModel::setMotion(const tf::Pose& pnew, const tf::Pose& pold)
    {
      tf::Pose delta_pose;
      tf::Transform odom_to_base(pold.inverse().getRotation());
      delta_pose.setOrigin(odom_to_base * (pnew.getOrigin() - pold.getOrigin()));
      delta_pose.setRotation(pnew.getRotation() * pold.getRotation().inverse());

      delta_x = delta_pose.getOrigin().x();
      delta_y = delta_pose.getOrigin().y();
      delta_yaw = atan2(sin(tf::getYaw(delta_pose.getRotation())), cos(tf::getYaw(delta_pose.getRotation())));

	  dxy=sqrt(delta_x*delta_x+delta_y*delta_y);
    }

  double MotionModel::normalize(double z)
    {
      return atan2(sin(z),cos(z));
    }

  double MotionModel::angle_diff(double a, double b)
    {
      double d1, d2;
      a = normalize(a);
      b = normalize(b);
      d1 = a-b;
      d2 = 2*M_PI - fabs(d1);
      if(d1 > 0)
        d2 *= -1.0;
      if(fabs(d1) < fabs(d2))
        return(d1);
      else
        return(d2);
    }

  tf::Pose MotionModel::updateAction(tf::Pose& p)
    {
      double delta_rot1;
      if (dxy < 0.01)
        delta_rot1 = 0.0;
      else
        delta_rot1 = angle_diff(atan2(delta_y, delta_x), delta_yaw);

      double delta_trans = dxy;
      double delta_rot2 = angle_diff(delta_yaw, delta_rot1);

      double delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)), fabs(angle_diff(delta_rot1, M_PI)));
      double delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)), fabs(angle_diff(delta_rot2, M_PI)));

      double delta_rot1_hat = angle_diff(delta_rot1, sampleGaussian(alpha1 * delta_rot1_noise*delta_rot1_noise + 
                                                                    alpha2 * delta_trans*delta_trans));
      double delta_trans_hat = delta_trans - sampleGaussian(alpha3 * delta_trans*delta_trans +
                                                            alpha4 * delta_rot1_noise*delta_rot1_noise +
                                                            alpha4 * delta_rot2_noise*delta_rot2_noise);
      double delta_rot2_hat = angle_diff(delta_rot2, sampleGaussian(alpha1 * delta_rot2_noise*delta_rot2_noise +
                                                                    alpha2 * delta_trans*delta_trans));

      delta_x = delta_trans_hat * cos(delta_rot1_hat);
      delta_y = delta_trans_hat * sin(delta_rot1_hat);
      delta_yaw = delta_rot1_hat + delta_rot2_hat;

      tf::Pose noisy_pose(tf::createQuaternionFromRPY(0,0,delta_yaw),tf::Vector3(delta_x,delta_y,0));
      tf::Transform base_to_global_ = tf::Transform(p.getRotation());
      noisy_pose.setOrigin(base_to_global_ * noisy_pose.getOrigin());
      p.setOrigin(p.getOrigin() + noisy_pose.getOrigin());
      p.setRotation(p.getRotation() * noisy_pose.getRotation());
    }
};
