/* Reads velocity (linear and angular) commands from move_base node and then publishes it to topic,
 * that the Arduino servo controller will subscribe to. 
 * 
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the 
 *     names of its contributors may be used to endorse or promote products 
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "std_msgs/UInt32.h"

#include <sstream>

std_msgs::UInt32 navigation;
std_msgs::Uint32 navigation;
// time in seconds since last joy message received before automatic stop
const double noMessageThreshold = 2;
double timeSinceLastMessage = noJoyMessageThreshold;
// make sure this is the same as in the Arduino
const double runFrequency = 60;
int steeringPosition = 90;
int ThrottlePosition = 100;

double maxVelocity = 0.5;
double maxSteering = 0.005;
bool flag = false;

/**********************************************************************************
 If no signal is received from the XBox controller after a while due to a disconnect 
 or error, "steering 90 throttle 110" is sent to the Arduino to instruct it to turn
 the car straight and brake. 
 **********************************************************************************/
void stopCallback()
{
  navigation.data = 90100;
}

void joystickCallback(const std_msgs::UInt32MultiArray::ConstPtr& msg)
{
 if (msg.flag == 1){
    flag = !flag;
 }
}

/**********************************************************************************
 The XBox controller outputs a (class?) of button, joystick, and lever position data.
 In here, we call it msg, and extract the data corresponding to the positions of the 
 steering joystick and acceleration lever. 
 This callback only executes upon receipt of message from joystick, and does not 
 directly send a message to the Arduino. It only modifies the message sent. 
 **********************************************************************************/
void navigationCallback(const ackermann_msgs::AckermannDrive.::ConstPtr& msg) 
{
 
  
  int steering_angle = msg->steering_angle;
  int steering_angle_velocity = msg->steering_angle_velocity;
  int translational_speed = msg->speed;
  int translational_acceleration = msg->acceleration;
  int jerk = msg->jerk;
  
  
  //To DO: convert steering_angle_velocity and acceleration values to the same scale as the arduino (94 to 100)
  // for normal forward operation. the two integers must be declared in this scope. 
  int steeringPosition = steeringPosition + msg->steering_angle_velocity;
  int throttlePosition = throttlePosition + msg->acceleration;
  
  // first three digits steering, last three digits throttle
  navigation.data = (steeringJoystickPosition * 1000) + throttleLeverPosition;

  timeSinceLastMessage = 0;
}

int main(int argc, char **argv)
{
  /**********************************************************************************
   The ros::init() function needs to see argc and argv so that it can perform
   any ROS arguments and name remapping that were provided at the command line.
   For programmatic remappings you can use a different version of init() which takes
   remappings directly, but for most command-line programs, passing argc and argv is
   the easiest way to do it.  The third argument to init() is the name of the node.
   
   You must call one of the versions of ros::init() before using any other
   part of the ROS system.
   **********************************************************************************/
  ros::init(argc, argv, "car_control");

  /**********************************************************************************
   NodeHandle is the main access point to communications with the ROS system.
   The first NodeHandle constructed will fully initialize this node, and the last
   NodeHandle destructed will close down the node.
   **********************************************************************************/
  ros::NodeHandle n;

  /**********************************************************************************
   Subscribes to topic joy, which the joystick publishes button and lever data to. 
   On receipt of data, executes function joystickCallback, which converts joystick
   data to a UInt8 which will be sent to the Arduino. 
   **********************************************************************************/
  ros::Subscriber sub = n.subscribe("ackermann", 1000, navigationCallback);
  ros::Subscriber sub = n.subscribe("cinnabar", 1000, joystickCallback);

  /**********************************************************************************
   The advertise() function is how you tell ROS that you want to publish on a given 
   topic name. This invokes a call to the ROS master node, which keeps a registry of 
   who is publishing and who is subscribing. After this advertise() call is made, 
   the master node will notify anyone who is trying to subscribe to this topic name,
   and they will in turn negotiate a peer-to-peer connection with this node.  
   advertise() returns a Publisher object which allows you to publish messages on 
   that topic through a call to publish(). Once all copies of the returned Publisher
   object are destroyed, the topic will be automatically unadvertised.
   
   The second parameter to advertise() is the size of the message queue used for
   publishing messages.  If messages are published more quickly than we can send them,
   the number here specifies how many messages to buffer up before throwing some away.
   **********************************************************************************/
  ros::Publisher pub = n.advertise<std_msgs::UInt32>("auto", 1000);

  ros::Rate loop_rate(runFrequency);

  while (ros::ok())
  {
    // DEBUG
    ROS_INFO("%u", navigation.data);

    if (flag){
    timeSinceLastMessage = timeSinceLastMessage + (1 / runFrequency);
    
    if (timeSinceLastMessage > noMessageThreshold) 
      stopCallback();

}
    /*******************************************************************************
     The publish() function is how you send messages. The parameter is the message
     object. The type of this object must agree with the type given as a template
     parameter to the advertise<>() call, as was done in the constructor above.
     *******************************************************************************/
    pub.publish(navigation);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
