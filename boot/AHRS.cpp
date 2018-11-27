#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <iostream>
#include <stdio.h>
#include <memory>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <Common/MPU9250.h>
#include <Navio2/LSM9DS1.h>
#include <Common/Util.h>
#include "AHRS.hpp"
#include <stdexcept>

const float G_SI = 9.80665f;
const float DT = 1/1300.0f;
const float MILLION = 1000000.0f;

AHRS::AHRS(std::string sensor_name, ros::NodeHandle &nh)
{
  if (sensor_name == "mpu") {
    printf("Selected: MPU9250\n");
    sensor = std::unique_ptr <InertialSensor>{ new MPU9250() };
  }
  else if (sensor_name == "lsm") {
    printf("Selected: LSM9DS1\n");
    sensor = std::unique_ptr <InertialSensor>{ new LSM9DS1() };
  }

  if (!sensor)
    throw(std::invalid_argument("Wrong sensor name. Select: mpu or lsm"));

  if (!sensor->probe())
    throw(std::runtime_error("Sensor not enabled"));

  msg.header.frame_id = sensor_name;
  pub_imu = nh.advertise<sensor_msgs::Imu>(sensor_name, 1);
}

void AHRS::publish() const
{
  pub_imu.publish(msg);
}

void AHRS::update(float dt)
{
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  sensor->update();
  sensor->read_accelerometer(&ax, &ay, &az);
  sensor->read_gyroscope(&gx, &gy, &gz);
  sensor->read_magnetometer(&mx, &my, &mz);


  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    updateIMU(dt);
    return;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = w_ * w_;
    q0q1 = w_ * x_;
    q0q2 = w_ * y_;
    q0q3 = w_ * z_;
    q1q1 = x_ * x_;
    q1q2 = x_ * y_;
    q1q3 = x_ * z_;
    q2q2 = y_ * y_;
    q2q3 = y_ * z_;
    q3q3 = z_ * z_;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of gravity and magnetic field
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;	// apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f;	// prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);		// pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = w_;
  qb = x_;
  qc = y_;
  w_ += (-qb * gx - qc * gy - z_ * gz);
  x_ += (qa * gx + qc * gz - z_ * gy);
  y_ += (qa * gy - qb * gz + z_ * gx);
  z_ += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);
  w_ *= recipNorm;
  x_ *= recipNorm;
  y_ *= recipNorm;
  z_ *= recipNorm;
}

void AHRS::updateIMU(float dt)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  float ax, ay, az;
  float gx, gy, gz;

  // Accel + gyro.
  sensor->update();
  sensor->read_accelerometer(&ax, &ay, &az);
  sensor->read_gyroscope(&gx, &gy, &gz);

  ax /= G_SI;
  ay /= G_SI;
  az /= G_SI;
  gx *= (180 / M_PI) * 0.0175;
  gy *= (180 / M_PI) * 0.0175;
  gz *= (180 / M_PI) * 0.0175;

  gx -= gyroOffset[0];
  gy -= gyroOffset[1];
  gz -= gyroOffset[2];

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = x_ * z_ - w_ * y_;
    halfvy = w_ * x_ + y_ * z_;
    halfvz = w_ * w_ - 0.5f + z_ * z_;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;	// apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f;	// prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);		// pre-multiply common factors
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = w_;
  qb = x_;
  qc = y_;
  w_ += (-qb * gx - qc * gy - z_ * gz);
  x_ += (qa * gx + qc * gz - z_ * gy);
  y_ += (qa * gy - qb * gz + z_ * gx);
  z_ += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);
  w_ *= recipNorm;
  x_ *= recipNorm;
  y_ *= recipNorm;
  z_ *= recipNorm;


  //ROS Message publisher ----------------------
  msg.angular_velocity.x=static_cast<double>(gx);
  msg.angular_velocity.y=static_cast<double>(gy);
  msg.angular_velocity.z=static_cast<double>(gz);
  msg.linear_acceleration.x=static_cast<double>(-ax*G_SI);
  msg.linear_acceleration.y=static_cast<double>(-ay*G_SI);
  msg.linear_acceleration.z=static_cast<double>(-az*G_SI);
  // --------------------------------------------

}

void AHRS::setGyroOffset()
{
  //---------------------- Calculate the offset -----------------------------

  float offset[3] = {0.0, 0.0, 0.0};
  float gx, gy, gz;

  //----------------------- MPU initialization ------------------------------

  sensor->initialize();

  //-------------------------------------------------------------------------

  printf("Beginning Gyro calibration...\n");
  for(int i = 0; i<100; i++)
  {
    sensor->update();
    sensor->read_gyroscope(&gx, &gy, &gz);

    gx *= 180 / M_PI;
    gy *= 180 / M_PI;
    gz *= 180 / M_PI;

    offset[0] += gx*0.0175f;
    offset[1] += gy*0.0175f;
    offset[2] += gz*0.0175f;

    usleep(10000);
  }
  offset[0]/=100.0;
  offset[1]/=100.0;
  offset[2]/=100.0;

  std::cout << "Offsets are: " << offset[0] << " " << offset[1] << " " << offset[2] << '\n';

  gyroOffset[0] = offset[0];
  gyroOffset[1] = offset[1];
  gyroOffset[2] = offset[2];
}

float AHRS::invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void print_help()
{
  printf("Possible parameters:\nSensor selection: -i [sensor name]\n");
  printf("Sensors names: mpu is MPU9250, lsm is LSM9DS1\nFor help: -h\n");
  printf("If you want to visualize IMU data on another machine,\n");
  printf("add IP address and port number (by default 7000):\n");
  printf("-i [sensor name] ipaddress portnumber\n");

}


//============================== Main loop ====================================

void AHRS::computeOrientation()
{
  // Orientation data
  struct timeval tv;
  float dt;
  // Timing data

  static float maxdt;
  static float mindt(0.01f);
  static float dtsumm(0);
  static int isFirst(1);
  static unsigned long previoustime, currenttime;


  //----------------------- Calculate delta time ----------------------------

  gettimeofday(&tv,nullptr);
  previoustime = currenttime;
  currenttime = static_cast<ulong>(MILLION * tv.tv_sec + tv.tv_usec);
  dt = (currenttime - previoustime) / MILLION;
  if(dt < DT) usleep((DT-dt)*MILLION);
  gettimeofday(&tv,nullptr);
  currenttime = static_cast<ulong>(MILLION * tv.tv_sec + tv.tv_usec);
  dt = (currenttime - previoustime) / MILLION;

  //-------- Read raw measurements from the MPU and update AHRS --------------
  updateIMU(dt);

  //------------------- Discard the time of the first cycle -----------------


  if (!isFirst)
  {
    if (dt > maxdt) maxdt = dt;
    if (dt < mindt) mindt = dt;
  }
  isFirst = 0;

  //------------- Console and network output ------------

  dtsumm += dt;
  if(dtsumm > 0.05f)
  {
    msg.orientation.x=static_cast<double>(x_);
    msg.orientation.y=static_cast<double>(y_);
    msg.orientation.z=static_cast<double>(z_);
    msg.orientation.w=static_cast<double>(w_);
    dtsumm = 0;
  }
}

std::string imuName()
{
  std::string name = "";
  if (ros::NodeHandle("~").getParam("sensor_name", name))
    ROS_INFO("Got param: %sensor_name", name.c_str());
  else
    ROS_ERROR("Failed to get param 'sensor_name'");
  return name;
}

//=============================================================================

int main(int argc, char *argv[])
{
  if (check_apm()) {
    return 1;
  }

  // declare node and loop rate at 100 Hz
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle nh;
  ros::Rate loop(100);

  // setup AHRS
  auto ahrs = std::unique_ptr <AHRS>{new AHRS(imuName(), nh)};
  ahrs->setGyroOffset();

  while(ros::ok())
  {
    ahrs->computeOrientation();
    ahrs->publish();
    ros::spinOnce();
    loop.sleep();
  }
}
