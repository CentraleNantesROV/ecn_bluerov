#include <unistd.h>
#include <cstdio>
#include <Common/Util.h>
#include <Navio2/ADC_Navio2.h>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <ecn_bluerov/ADC.h>

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
    
    ecn_bluerov::ADC msg;
    ros::Publisher pub_tension=nh.advertise<ecn_bluerov::ADC>("tensions",1);
    auto adc = std::unique_ptr <ADC>{ new ADC_Navio2() };
    adc->initialize();

    const std::vector<float*> msg_ptr = {&msg.board,
                                        &msg.rail,
                                        &msg.power_voltage,
                                        &msg.power_current,
                                        &msg.leak,
                                        &msg.adc3};
    while (ros::ok())
    {
        for (int i = 0; i < adc->get_channel_count(); i++)
        {
            const float value = adc->read(i);
            if(value == READ_FAILED)
                return EXIT_FAILURE;
            *(msg_ptr[i]) = value;
        }
        pub_tension.publish(msg);
        
        usleep(500000);
    }


    return 0;
}
