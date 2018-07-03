#include <unistd.h>
#include <Navio2/PWM.h>
#include <Navio2/RCOutput_Navio2.h>
#include <Common/Util.h>
#include <unistd.h>
#include <memory>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>

std::vector<double> thruster_force;
double tilt_angle;
double thruster_time;
double tilt_time;

double interp(double v, const std::vector<double> &x, const std::vector<double> &y)
{
    // no extrapolation
    if(v <= x[0])
        return y[0];
    else if(v >=x.back())
        return y.back();

    int i = 0;
    while ( v > x[i+1] ) i++;

    const double xL = x[i], yL = y[i], xR = x[i+1], yR = y[i+1];

    double dydx = ( yR - yL ) / ( xR - xL );

    return yL + dydx * ( v - xL );
}



void readThrusters(const sensor_msgs::JointStateConstPtr &msg)
{
    const std::vector<std::string> names = {"thr1", "thr2", "thr3", "thr4", "thr5", "thr6"};
    thruster_time = ros::Time::now().toSec();
    for(int i = 0; i<msg->name.size();++i)
    {
        for(int j = 0; j<6; ++j)
        {
            if(msg->name[i] == names[j])
            {
                thruster_force[j] = msg->effort[i];
                break;
            }
        }
    }
}

void readTilt(const sensor_msgs::JointStateConstPtr &msg)
{
    if(msg->name.size() && msg->position.size())
    {
        std::cout << "read tilt command: " << msg->position[0] << std::endl;
        tilt_time = ros::Time::now().toSec();
        tilt_angle = msg->position[0];
        if(std::abs(tilt_angle) > 0.785)
            tilt_angle = 0.785 * tilt_angle/std::abs(tilt_angle);
    }
}

int main(int argc, char *argv[])
{
    if (check_apm()) {
        return 1;
    }

    if (getuid()) {
        fprintf(stderr, "Not root. Please launch like this: sudo %s\n", argv[0]);
    }

    // the PWM we control
    const std::vector<int> pwm_idx = {0, 2, 4, 6, 8, 10, 12};
    // wait for PWM
    auto pwm = std::unique_ptr <RCOutput_Navio2>{new RCOutput_Navio2()};

    // activate all outputs
    for(auto idx: pwm_idx)
    {
        if( !(pwm->initialize(idx)) )
            return 1;
        pwm->set_frequency(idx, 50);
        if ( !(pwm->enable(idx)))
            return 1;
    }

    // arm
    for(auto idx: pwm_idx)
    {
        pwm->set_duty_cycle(idx, 1500);
        std::cout << "PWM # " << idx << " armed\n";
        sleep(1);
    }

    ros::init(argc, argv, "pwm_node");
    ros::NodeHandle nh;
    thruster_force.resize(6, 0);
    tilt_angle = 0;
    thruster_time = tilt_time = 0;
    ros::Rate loop(50);

    ros::Subscriber thruster_sub = nh.subscribe("thruster_command", 10,
                                                readThrusters);
    ros::Subscriber tilt_sub = nh.subscribe("tilt_command", 10,
                                            readTilt);

    // maps
    const std::vector<double> forces = {-30, 0, 30};
    const std::vector<double> forces_pwm = {1100, 1500, 1900};

    while (ros::ok())
    {
        const double t= ros::Time::now().toSec();
        if(t - thruster_time > 1)
            thruster_force = {0,0,0,0,0,0};
        if(t - tilt_time > 1)
            tilt_angle = 0;

        // apply thruster pwm
        for(int i = 0; i < 6; ++i)
        {
            const double v = interp(thruster_force[i], forces, forces_pwm);
            //std::cout << "Thr# " << i << ", force = " << thruster_force[i] << ", pwm = " << v << std::endl; 
            pwm->set_duty_cycle(pwm_idx[i], v);
        }
        // tilt pwm
        pwm->set_duty_cycle(pwm_idx[6], 1500 + 509.3*tilt_angle);
        //std::cout << "Tilt, angle = " << tilt_angle << ", pwm = " << 1100 + 509.3*(tilt_angle + 0.785) << std::endl; 

        ros::spinOnce();
        loop.sleep();
    }
    
    // stop pwms
    for(auto idx: pwm_idx)
            pwm->set_duty_cycle(idx, 1500);
    sleep(3);

    return 0;
}
