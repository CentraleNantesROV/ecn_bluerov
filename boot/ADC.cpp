#include <unistd.h>
#include <cstdio>
#include <Common/Util.h>
#include <Navio2/ADC_Navio2.h>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#define READ_FAILED -1

int main(int argc, char *argv[])
{
    if (check_apm()) {
        return 1;
    }
    //declare node and loop rate at 10 hz
    ros::init(argc, argv, "ADC_node");
    ros::NodeHandle nh;
    ros::Rate loop(10);
    
    std_msgs::Float32MultiArray msg_tension;
    ros::Publisher pub_tension=nh.advertise<std_msgs::Float32MultiArray>("tensions",1);
    auto adc = std::unique_ptr <ADC>{ new ADC_Navio2() };
    adc->initialize();
    msg_tension.data.resize(adc->get_channel_count());
        
    while (ros::ok())
    {
        for (int i = 0; i < adc->get_channel_count(); i++)
        {
            
            msg_tension.data[i] = adc->read(i);
            if(msg_tension.data[i] == READ_FAILED)
                return EXIT_FAILURE;
        }
        pub_tension.publish(msg_tension);
        
        usleep(500000);
    }


    return 0;
}
