#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include "pwm_sensor_reader.hpp"

#include <pigpiod_if2.h>

PwmSensorReader::PwmSensorReader(int in_pi, std::string prefix, PwmSensorReaderCB_t in_callback)
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

void PwmSensorReader::_callbackExt(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user)
{
   /*
      Need a static callback to link with C.
   */

   PwmSensorReader *mySelf = (PwmSensorReader *) user;

   mySelf->_callback(level, tick); /* Call the instance callback. */
}

void PwmSensorReader::_callback(unsigned in_level, uint32_t tick)
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

double PwmSensorReader::remap_value(uint32_t value) 
{
    double middle = ((double)(range_max + range_min))/2.0;
    double amplitude = range_max - middle;
    return (value - middle)/amplitude;
}

double PwmSensorReader::getSensitivity()
{
    return sensitivity;
}

