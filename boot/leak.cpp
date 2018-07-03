#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <iostream>
#include <stdio.h>

#include <unistd.h>
#include <cstdio>

#include <Navio2/RCInput_Navio2.h>
#include <Common/Util.h>
#include <memory>

#define READ_FAILED -1

std_msgs::Float32MultiArray msg;

std::unique_ptr <RCInput> get_rcin()
{
        auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio2() };
        return ptr;
}


int main(int argc, char *argv[])
{
    if (check_apm()) {
        return 1;
    }

// declare node and loop rate at 10 Hz
    ros::init(argc, argv, "leak_node");
    ROS_INFO("ros connected to pirov_leak");
    ros::NodeHandle nh_("~");
    ros::Rate loop(10);


    auto rcin = get_rcin();
    rcin->initialize();

//Publishing
    ros::Publisher pub_leak = nh_.advertise<std_msgs::Float32MultiArray>("leak_topic", 1);

    while (ros::ok())
    {
   for (int i=0;i<7;i++){
        float period = rcin->read(0);
        if (period == READ_FAILED)
            return EXIT_FAILURE;
        
        msg.data.push_back(period);    
        pub_leak.publish(msg);        
        printf("%d ",period);      

   }      
   msg.data.clear(); 
    sleep(1);
    }

    return 0;
}
