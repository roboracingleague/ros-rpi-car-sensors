#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "pwm_sensor_reader.hpp"

#include <pigpiod_if2.h>

ros::Publisher odom_publisher;

class BrushlessOdomReader
{
    int pi, gpio, cb_id; 
    int range_min = 0, range_max = 500;
    uint32_t occurences = 0;
    double last_exec = 0.0;

    void _callback(unsigned level, uint32_t tick);

    /* Need a static callback to link with C. */
    static void _callbackExt(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user);
    double remap_value(double value); 

    public:

    BrushlessOdomReader(int pi, std::string prefix);
    
    double update();
};

BrushlessOdomReader::BrushlessOdomReader(int in_pi, std::string prefix)
{
    ros::NodeHandle pH("~" + prefix);

    pi = in_pi;
    pH.getParam("gpio", gpio);
    pH.param("range_max", range_max, 500);
    pH.param("range_min", range_min, 0);

    set_mode(pi, gpio, PI_INPUT);
    cb_id = callback_ex(pi, gpio, EITHER_EDGE, _callbackExt, (void *)this);
}

void BrushlessOdomReader::_callbackExt(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user)
{
   /*
      Need a static callback to link with C.
   */

   BrushlessOdomReader *mySelf = (BrushlessOdomReader *) user;

   mySelf->_callback(level, tick); /* Call the instance callback. */
}

void BrushlessOdomReader::_callback(unsigned in_level, uint32_t tick)
{
    occurences++;
}

double BrushlessOdomReader::remap_value(double value) 
{
    return ((double)(value - range_min))/((double)(range_max - range_min));
}


double BrushlessOdomReader::update()
{
    double crt = time_time();
    double occ = occurences;
    
    double odom = occ/(crt - last_exec);
    if (last_exec == 0.0) odom = 0.0;

    occurences = 0;
    last_exec = crt;

    return remap_value(odom);
}

void publish(double value)
{
    std_msgs::Float64 odom;
    odom.data = value;
    odom_publisher.publish(odom);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "brushless_odom");
    ros::NodeHandle n;
    ros::NodeHandle pH("~odom");

    // Init GPIO
    int pi = pigpio_start(NULL, NULL);

    int power_pin, p_loop_rate = 10;
    pH.param("loop_rate", p_loop_rate, 10);
    pH.getParam("power", power_pin);

    set_mode(pi, power_pin, PI_OUTPUT);
    gpio_write(pi, power_pin, 1);

    odom_publisher = n.advertise<std_msgs::Float64>("/odom", 1);

    BrushlessOdomReader odom(pi, "odom");

    ros::Rate loop_rate(p_loop_rate); 

    while (ros::ok())
    {
        ros::spinOnce();

        publish(odom.update());

        loop_rate.sleep();
    }
    // Handle ROS communication events

    pigpio_stop(pi);

    return 0;
}
