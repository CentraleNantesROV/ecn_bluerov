#include <unistd.h>
#include <cstdio>
#include <Common/Util.h>
#include <Navio2/ADC_Navio2.h>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#define READ_FAILED -1



std::unique_ptr <ADC> get_converter()
{

        auto ptr = std::unique_ptr <ADC>{ new ADC_Navio2() };
    
        return ptr;

}


int main(int argc, char *argv[])
{
    if (check_apm()) {
        return 1;
    }
    //declare node and loop rate at 10 hz
    ros::init(argc, argv, "ADC_node");
    ROS_INFO("ros connected to pirov_ADC");
    ros::NodeHandle nh_("~");
    ros::Rate loop(10);
    
    std_msgs::Float32MultiArray msg_tension;
    ros::Publisher pub_tension=nh_.advertise<std_msgs::Float32MultiArray>("tension",1);                
    auto adc = get_converter();
    adc->initialize();
    float result;
        
    while (ros::ok())
    {
        for (int i = 0; i < adc->get_channel_count(); i++)
        {
            
            result = adc->read(i);
            msg_tension.data.push_back(result);
            if (result == READ_FAILED)
                return EXIT_FAILURE;
        }
        pub_tension.publish(msg_tension);
        msg_tension.data.clear();
        
        usleep(500000);
    }


    return 0;
}
