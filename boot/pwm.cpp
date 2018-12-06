#include <unistd.h>
#include <Navio2/PWM.h>
#include <Navio2/RCOutput_Navio2.h>
#include <Common/Util.h>
#include <unistd.h>
#include <memory>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/JointState.h>

float interp(float v, const std::vector<float> &x, const std::vector<float> &y)
{
  // no extrapolation
  if(v <= x[0])
    return y[0];
  else if(v >=x.back())
    return y.back();

  uint i = 0;
  while ( v > x[i+1] ) i++;

  const float xL = x[i], yL = y[i], xR = x[i+1], yR = y[i+1];

  return yL + ( yR - yL ) / ( xR - xL ) * ( v - xL );
}

class Listener
{
public:
  Listener(ros::NodeHandle &nh, std::string thruster_topic, std::string tilt_topic)
  {
    thruster_sub = nh.subscribe(thruster_topic, 10, &Listener::readThrusters, this);
    tilt_sub = nh.subscribe(tilt_topic, 10, &Listener::readTilt, this);
    thruster_force.resize(6, 0);
  }

  float thrust(uint i) const
  {
    return static_cast<float>(thruster_force[i]);
  }
  double tilt() const
  {
    return tilt_angle;
  }

  void checkSilent(double t)
  {
    if(t - thruster_time > 1)
    {
      for(auto & thruster: thruster_force)
        thruster = 0;
    }
    if(t - tilt_time > 1)
      tilt_angle = 0;
  }

private:
  std::vector<double> thruster_force;
  double tilt_angle = 0;
  double thruster_time = 0;
  double tilt_time = 0;
  ros::Subscriber thruster_sub, tilt_sub;

  void readThrusters(const sensor_msgs::JointStateConstPtr &msg)
  {
    const static std::vector<std::string> names = {"thr1", "thr2", "thr3", "thr4", "thr5", "thr6"};
    thruster_time = ros::Time::now().toSec();
    for(uint i = 0; i<msg->name.size();++i)
    {
      for(uint j = 0; j<6; ++j)
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
      tilt_time = ros::Time::now().toSec();
      tilt_angle = std::min(0.785, std::max(-0.785, msg->position[0]));
    }
  }
};


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
    sleep(1);
  }

  ros::init(argc, argv, "pwm_node");
  ros::NodeHandle nh;
  ros::Rate loop(50);
  Listener listener(nh, "thruster_command", "joint_setpoint");
  ros::Publisher tilt_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  sensor_msgs::JointState tilt_msg;
  tilt_msg.name = {"tilt"};
  tilt_msg.position = {0};

  // maps
  const std::vector<float> forces = {-40, -35.5, -28.5, -22.2, -16.1, -10, -5.1, -1.1, 0, 0, 1.8, 6.2, 12.2, 19.2, 26.4, 33.8, 41.1, 49.9};
  const std::vector<float> forces_pwm = {1100, 1150, 1200, 1250, 1300, 1350, 1400, 1450, 1480, 1520, 1550, 1600, 1650, 1700, 1750, 1800, 1850, 1900};

  while (ros::ok())
  {
    const double t = ros::Time::now().toSec();
    listener.checkSilent(t);

    // apply thruster pwm
    for(uint i = 0; i < 6; ++i)
    {
      const float v = interp(listener.thrust(i), forces, forces_pwm);
      //std::cout << "Thr# " << i << ", force = " << thruster_force[i] << ", pwm = " << v << std::endl;
      pwm->set_duty_cycle(pwm_idx[i], v);
    }
    // tilt pwm
    pwm->set_duty_cycle(pwm_idx[6], 1500 + 509.3*listener.tilt());
    //std::cout << "Tilt, angle = " << tilt_angle << ", pwm = " << 1100 + 509.3*(tilt_angle + 0.785) << std::endl;

    tilt_msg.position[0] = listener.tilt();
    tilt_msg.header.stamp = ros::Time::now();
    tilt_pub.publish(tilt_msg);

    ros::spinOnce();
    loop.sleep();
  }

  // stop pwms
  for(auto idx: pwm_idx)
    pwm->set_duty_cycle(idx, 1500);
  sleep(3);

  return 0;
}
