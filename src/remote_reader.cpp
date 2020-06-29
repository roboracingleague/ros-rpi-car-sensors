#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include <pigpiod_if2.h>

typedef void (*RemoteReaderCB_t)(double);

class RemoteReader
{
    int pi, gpio, level = 1, cb_id;
    int range_max = 2000, range_min = 1000;
    double sensitivity = 0.015;
    double last_value = 0.0;
    RemoteReaderCB_t callback;

    uint32_t start_tick, end_tick;

    void _callback(unsigned level, uint32_t tick);

    double remap_value(uint32_t value);

    /* Need a static callback to link with C. */
    static void _callbackExt(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user);

    public:
    
    RemoteReader(int pi, std::string prefix, RemoteReaderCB_t callback);
    
    double getSensitivity();
};

RemoteReader::RemoteReader(int in_pi, std::string prefix, RemoteReaderCB_t in_callback)
{
    ros::NodeHandle pH("~" + prefix);

    pi = in_pi;
    pH.getParam("gpio", gpio);
    pH.param("sensitivity", sensitivity, 0.015);
    pH.param("level", level, 1);
    pH.param("range_max", range_max, 2000);
    pH.param("range_min", range_min, 1000);
    callback = in_callback;

    set_mode(pi, gpio, PI_INPUT);
    cb_id = callback_ex(pi, gpio, EITHER_EDGE, _callbackExt, (void *)this);
}

void RemoteReader::_callbackExt(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user)
{
   /*
      Need a static callback to link with C.
   */

   RemoteReader *mySelf = (RemoteReader *) user;

   mySelf->_callback(level, tick); /* Call the instance callback. */
}

void RemoteReader::_callback(unsigned in_level, uint32_t tick)
{
    if (in_level == level)
    {
        start_tick = tick;
    } else {
        end_tick = tick;
        uint32_t diff = end_tick - start_tick;

        double remaped = remap_value(tick - start_tick);
        if (fabs(remaped - last_value) > sensitivity)
        {
            last_value = remaped;
            callback(remaped);
        }
    }
}

double RemoteReader::remap_value(uint32_t value) 
{
    double middle = ((double)(range_max + range_min))/2.0;
    double amplitude = range_max - middle;
    return (value - middle)/amplitude;
}

double RemoteReader::getSensitivity()
{
    return sensitivity;
}


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

    RemoteReader steering(pi, "steering", steering_cb);
    RemoteReader throttle(pi, "throttle", throttle_cb);
    RemoteReader ch3(pi, "channel3", channel3_cb);

    // Handle ROS communication events
    ros::spin();

    pigpio_stop(pi);

    return 0;
}
