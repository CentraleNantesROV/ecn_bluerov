/*
Mahony AHRS algorithm implemented by Madgwick
See: http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

Adapted by Igor Vereninov (igor.vereninov@emlid.com)
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com
*/

#ifndef AHRS_HPP
#define AHRS_HPP

#include <cmath>
#include <stdio.h>
#include <memory>
#include <Common/InertialSensor.h>
#include <sensor_msgs/Imu.h>
#include <ros/publisher.h>

class AHRS{
private:
  float w_=1, x_=0, y_=0, z_=0;
	float gyroOffset[3];
  float twoKi=0;
  float twoKp=2;
	float integralFBx, integralFBy, integralFBz;

  std::unique_ptr <InertialSensor> sensor;
  ros::Publisher pub_imu;

public:
    AHRS(std::string name, ros::NodeHandle &nh);

    void update(float dt);
    void updateIMU(float dt);
    void setGyroOffset();
    void computeOrientation();
    void publish() const;

    float invSqrt(float x);

    sensor_msgs::Imu msg;
};

#endif // AHRS_hpp
