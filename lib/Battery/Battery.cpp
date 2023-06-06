#include "Battery.h"

uint8_t readBattery(AnalogIn *battery_reader)
{
    float full_battery_voltage = 12.6;
    float empty_battery_voltage = 9.6;

    // ~0.76, indicates the level in [0,1] at which the battery is considered empty, to use as
    //   reference as the 0% value
    float scaled_empty_battery_voltage = empty_battery_voltage / full_battery_voltage;

    // In range [0, 1], meaning in range [0v, 3.3v]
    float read_scaled_battery_voltage = battery_reader->read();

    // In range [0, 1], meaning in range [2.51v, 3.3v] from analog pin, meaning [9.6v, 12.6v] at battery
    // The percentage is calculated by subtracting the scaled empty battery value from read value,
    //   then dividing by the "usable" scaled battery voltage range (1 - scaled range)
    float battery_percentage = (read_scaled_battery_voltage - scaled_empty_battery_voltage) / 
                                (1 - scaled_empty_battery_voltage);

    return (uint8_t) (battery_percentage * 100);
}