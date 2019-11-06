
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>


#include <iostream>

#include <Common/MS5611.h>
#include <Common/Util.h>
#include <unistd.h>
#include <stdio.h>


int main(int argc, char *argv[])
{
    MS5611 barometer;

    if (check_apm()) {
        return 1;
    }

    // declare node and loop rate at 10 Hz
    ros::init(argc, argv, "barometer_node");
    ROS_INFO("ros connected to blurr_barometer");
    ros::NodeHandle nh_("~");
    ros::Rate loop(10);

    barometer.initialize();
    //Publishing
    ros::Publisher pub_temperature = nh_.advertise<sensor_msgs::Temperature>("temperature", 1);

    //Publishing
    ros::Publisher pub_pressure = nh_.advertise<sensor_msgs::FluidPressure>("pressure", 1);

    sensor_msgs::Temperature msg_temp;
    sensor_msgs::FluidPressure msg_pres;

    // wait a bit
    usleep(3000000);

    while (ros::ok()) {
        barometer.refreshPressure();
        usleep(10000); // Waiting for pressure data ready
        barometer.readPressure();

        barometer.refreshTemperature();
        usleep(10000); // Waiting for temperature data ready
        barometer.readTemperature();

        barometer.calculatePressureAndTemperature();

        //printf("Temperature(C): %f Pressure(millibar): %f\n", 
        //        barometer.getTemperature(), barometer.getPressure());

        msg_temp.header.stamp = msg_pres.header.stamp = ros::Time::now();

        msg_temp.temperature = barometer.getTemperature();
        msg_pres.fluid_pressure = barometer.getPressure();
       
        pub_temperature.publish(msg_temp);
        pub_pressure.publish(msg_pres);
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
