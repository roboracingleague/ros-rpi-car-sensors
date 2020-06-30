#ifndef __PWM_SENSOR_READER
#define __PWM_SENSOR_READER

typedef void (*PwmSensorReaderCB_t)(double);

class PwmSensorReader
{
    int pi, gpio, level = 1, cb_id;
    int range_max = 2000, range_min = 1000;
    double sensitivity = 0.015;
    double last_value = 0.0;
    PwmSensorReaderCB_t callback;

    uint32_t start_tick, end_tick;

    void _callback(unsigned level, uint32_t tick);

    double remap_value(uint32_t value);

    /* Need a static callback to link with C. */
    static void _callbackExt(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user);

    public:
    
    PwmSensorReader(int pi, std::string prefix, PwmSensorReaderCB_t callback);
    
    double getSensitivity();
};

#endif
