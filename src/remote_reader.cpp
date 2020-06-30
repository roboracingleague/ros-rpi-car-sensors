#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include "pwm_sensor_reader.hpp"

#include <pigpiod_if2.h>

ros::Publisher remote_publisher;
double steering = 0.0, throttle = 0.0;
int button_value = 0;

void publish_drive()
{
    sensor_msgs::Joy command;
    command.axes.resize(2);
    command.axes[1] = throttle;
    command.axes[0] = steering;
    command.buttons.resize(1);
    command.buttons[0] = button_value;

    remote_publisher.publish(command);
}

void steering_cb(double value)
{
    steering = value;
    publish_drive();
}

void throttle_cb(double value)
{
    throttle = value;
    publish_drive();
}

void channel3_cb(double value)
{
    button_value = (int)value;
    publish_drive();
}


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "remote_reader");
    ros::NodeHandle n;

    steering = 0.0;
    throttle = 0.0;

    remote_publisher = n.advertise<sensor_msgs::Joy>("/remote", 1);

    // Init GPIO
    int pi = pigpio_start(NULL, NULL);

    PwmSensorReader steering(pi, "steering", steering_cb);
    PwmSensorReader throttle(pi, "throttle", throttle_cb);
    PwmSensorReader ch3(pi, "channel3", channel3_cb);

    // Handle ROS communication events
    ros::spin();

    pigpio_stop(pi);

    return 0;
}
