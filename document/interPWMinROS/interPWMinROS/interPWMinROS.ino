

#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

geometry_msgs::Twist twist_msg;
ros::Publisher chatter("cmd_vel", &twist_msg);

volatile int pwm_g = 0;
volatile int pt_g = 0;
volatile int pwm_t = 0;
volatile int pt_t = 0;
 

void setup()
{
  nh.initNode();
  nh.advertise(chatter);

  attachInterrupt(0, rising_g, RISING); // pin2
  attachInterrupt(1, rising_t, RISING); // pin3
}

void rising_g() {
  attachInterrupt(0, falling_g, FALLING);
  pt_g = micros();
}
 
void falling_g() {
  attachInterrupt(0, rising_g, RISING);
  pwm_g = micros()-pt_g;
  twist_msg.linear.x = pwm_g;
}

void rising_t() {
  attachInterrupt(1, falling_t, FALLING);
  pt_t = micros();
}
 
void falling_t() {
  attachInterrupt(1, rising_t, RISING);
  pwm_t = micros()-pt_t;
  twist_msg.angular.z = pwm_t;
}
void loop()
{
  chatter.publish( &twist_msg );
  nh.spinOnce();
  delay(10);
}
