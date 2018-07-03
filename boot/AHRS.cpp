/*
This code is provided under the BSD license.
Copyright (c) 2014, Emlid Limited. All rights reserved.
Written by Igor Vereninov and Mikhail Avkhimenia
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Application: Mahory AHRS algorithm supplied with data from MPU9250 and LSM9DS1.
Outputs roll, pitch and yaw in the console and sends quaternion
over the network - it can be used with 3D IMU visualizer located in
Navio/Applications/3D IMU visualizer.

To run this app navigate to the directory containing it and run following commands:
make
sudo ./AHRS -i [sensor name] ipaddress portnumber
Sensors names: mpu is MPU9250, lsm is LSM9DS1.
If you want to visualize IMU data on another machine pass it's address and port
For print help:
./AHRS -h

To achieve stable loop you need to run this application with a high priority
on a linux kernel with real-time patch. Raspbian distribution with real-time
kernel is available at emlid.com and priority can be set with chrt command:
chrt -f -p 99 PID
*/
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

#define G_SI 9.80665
#define PI   3.14159

sensor_msgs::Imu msg_imu;


AHRS::AHRS(std::unique_ptr <InertialSensor> imu)
{
    sensor = move(imu);
    q0 = 1; q1 = 0; q2 = 0, q3 = 0; twoKi = 0; twoKp =2;
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
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

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
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
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
    gx *= (180 / PI) * 0.0175;
    gy *= (180 / PI) * 0.0175;
    gz *= (180 / PI) * 0.0175;

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
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

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
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;


    //ROS Message publisher ----------------------
    
    msg_imu.angular_velocity.x=gx;
    msg_imu.angular_velocity.y=gy;
    msg_imu.angular_velocity.z=gz;
    msg_imu.linear_acceleration.x=ax;
    msg_imu.linear_acceleration.y=ay;
    msg_imu.linear_acceleration.z=az;
    // ---------------------------------------------

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

        gx *= 180 / PI;
        gy *= 180 / PI;
        gz *= 180 / PI;

        offset[0] += gx*0.0175;
        offset[1] += gy*0.0175;
        offset[2] += gz*0.0175;

        usleep(10000);
    }
    offset[0]/=100.0;
    offset[1]/=100.0;
    offset[2]/=100.0;

    printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);

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

float AHRS::getW()
{
    return  q0;
}

float AHRS::getX()
{
    return  q1;
}

float AHRS::getY()
{
    return  q2;
}

float AHRS::getZ()
{
    return  q3;
}

std::unique_ptr <InertialSensor> get_inertial_sensor( std::string sensor_name)
{
    if (sensor_name == "mpu") {
        printf("Selected: MPU9250\n");
        auto ptr = std::unique_ptr <InertialSensor>{ new MPU9250() };
        return ptr;
    }
    else if (sensor_name == "lsm") {
        printf("Selected: LSM9DS1\n");
        auto ptr = std::unique_ptr <InertialSensor>{ new LSM9DS1() };
        return ptr;
    }
    else {
        return NULL;
    }
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

void imuLoop(AHRS* ahrs)
{
    // Orientation data

    struct timeval tv;
    float dt;
    // Timing data

    static float maxdt;
    static float mindt = 0.01;
    static float dtsumm = 0;
    static int isFirst = 1;
    static unsigned long previoustime, currenttime;


    //----------------------- Calculate delta time ----------------------------

	gettimeofday(&tv,NULL);
	previoustime = currenttime;
	currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;
	if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
        gettimeofday(&tv,NULL);
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;

    //-------- Read raw measurements from the MPU and update AHRS --------------

    ahrs->updateIMU(dt);


    //------------------- Discard the time of the first cycle -----------------


    if (!isFirst)
    {
    	if (dt > maxdt) maxdt = dt;
    	if (dt < mindt) mindt = dt;
    }
    isFirst = 0;

    //------------- Console and network output ------------

    dtsumm += dt;
    if(dtsumm > 0.05)
    {
        // Network output
        msg_imu.orientation.x=ahrs->getX();
        msg_imu.orientation.y=ahrs->getY();
        msg_imu.orientation.z=ahrs->getZ();
        msg_imu.orientation.w=ahrs->getW();
        dtsumm = 0;
    }
}

//=============================================================================

int main(int argc, char *argv[])
{
    if (check_apm()) {
        return 1;
    }

    // declare node and loop rate at 10 Hz
    ros::init(argc, argv, "imu_node");
    ROS_INFO("ros connected to pirov_imu");
    ros::NodeHandle nh_("~");
    ros::Rate loop(10);


    //recupere le nom du imu

    //PROBLEME!!!!! ne reconnait pas sensor_name comme parametre
    std::string sensor_name;

    if (nh_.getParam("sensor_name", sensor_name))
    {
      ROS_INFO("Got param: %sensor_name", sensor_name.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'sensor_name'");
    }


  auto imu = get_inertial_sensor(sensor_name);
    if (!imu) {
        printf("Wrong sensor name. Select: mpu or lsm\n");
        return EXIT_FAILURE;
    }

    if (!imu->probe()) {
        printf("Sensor not enable\n");
        return EXIT_FAILURE;
    }

    //Publishing
    ros::Publisher pub_imu = nh_.advertise<sensor_msgs::Imu>(sensor_name, 1);



    //--------------------------- Network setup -------------------------------

    auto ahrs = std::unique_ptr <AHRS>{new AHRS(move(imu)) };

    //-------------------- Setup gyroscope offset -----------------------------
   
    ahrs->setGyroOffset();
    msg_imu.header.frame_id="BLUEROV";

    while(ros::ok())
    {
        imuLoop(ahrs.get());
        pub_imu.publish(msg_imu);
        ros::spinOnce();        

    }
}
