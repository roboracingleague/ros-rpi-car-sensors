#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <pigpiod_if2.h>

typedef void (*RemoteReaderCB_t)(double);

class RemoteReader
{
    int pi, gpio, cb_id;
    RemoteReaderCB_t callback;

    uint32_t start_tick, end_tick;

    void _callback(unsigned level, uint32_t tick);

    double remap_value(uint32_t value);

    /* Need a static callback to link with C. */
    static void _callbackExt(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user);

    public:
    
    RemoteReader(int pi, int gpio, RemoteReaderCB_t callback);
};

RemoteReader::RemoteReader(int in_pi, int in_gpio, RemoteReaderCB_t in_callback)
{
    pi = in_pi;
    gpio = in_gpio;
    callback = in_callback;

    set_mode(pi, gpio, PI_INPUT);
    cb_id = callback_ex(pi, 17, EITHER_EDGE, _callbackExt, (void *)this);
}

void RemoteReader::_callbackExt(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user)
{
   /*
      Need a static callback to link with C.
   */

//   ROS_INFO("Into static cb");

   RemoteReader *mySelf = (RemoteReader *) user;

   mySelf->_callback(level, tick); /* Call the instance callback. */
}

void RemoteReader::_callback(unsigned level, uint32_t tick)
{
//    ROS_INFO("Into _callback");
    if (level == 1)
    {
        start_tick = tick;
    } else {
        end_tick = tick;
        uint32_t diff = end_tick - start_tick;

        callback(remap_value(tick - start_tick));
    }
}

double RemoteReader::remap_value(uint32_t value) 
{
    return (value - 1500.0)/500.0;
}



ros::Publisher remote_drive_publisher;
double steering, throttle;

void publish_drive()
{
    geometry_msgs::Twist command;
    command.linear.x = throttle;
    command.angular.z = steering;

    remote_drive_publisher.publish(command);
}

void steering_cb(double value)
{
//    ROS_INFO("Into steering_cb");
    if (fabs(steering - value) > 0.01)
    {
        steering = value;
        publish_drive();
    }
}

void throttle_cb(double value)
{
//    ROS_INFO("Into steering_cb");
    if (fabs(throttle - value) > 0.01)
    {
        throttle = value;
        publish_drive();
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "remote_reader");
    ros::NodeHandle n;

    steering = 0.0;
    throttle = 0.0;

    remote_drive_publisher = n.advertise<geometry_msgs::Twist>("/remote/drive", 1);

    // Init GPIO
    int pi = pigpio_start(NULL, NULL);

//    ROS_INFO("Create reader");
    RemoteReader steering(pi, 17, steering_cb);
    RemoteReader throttle(pi, 27, throttle_cb);

    // Handle ROS communication events
    ros::spin();

    pigpio_stop(pi);

    return 0;
}
