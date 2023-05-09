#ifndef CUSTOM_MOTOR_H
#define CUSTOM_MOTOR_H

#include "mbed.h"

/** Interface to control a standard DC motor 
 *
 * with the DRV8871 H-bridge using two PwmOuts
 * https://www.ti.com/lit/ds/symlink/drv8871.pdf?HQS=dis-dk-null-digikeymode-dsf-pf-null-wwe&ts=1683638692290&ref_url=https%253A%252F%252Fwww.ti.com%252Fgeneral%252Fdocs%252Fsuppproductinfo.tsp%253FdistId%253D10%2526gotoUrl%253Dhttps%253A%252F%252Fwww.ti.com%252Flit%252Fgpn%252Fdrv8871
 */
class Motor {
public:

    /** Create a motor control interface    
     *
     * @param in1 A PwmOut pin, driving the H-bridge "IN1" input
     * @param in2 A PwmOut pin, driving the H-bridge "IN2" input
     * @param isNCHTimer If N-channel of timer is used, reverse duty-cycle logic
     */
    Motor(PinName in1, PinName in2, bool isNCHTimer);
    
    /** Set the speed of the motor
     * 
     * @param speed Duty-cycle for pwm. Normalized value between -1.0 and 1.0, measured as percentage. E.g. -0.50 = 50% in reverse 
     */
    void speed(float speed);

protected:
    PwmOut _in1;
    PwmOut _in2;
    bool _isNCHTimer;

};

#endif