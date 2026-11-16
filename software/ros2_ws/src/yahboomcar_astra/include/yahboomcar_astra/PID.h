
#ifndef YAHBOOMCAR_ASTRA_PID_H
#define YAHBOOMCAR_ASTRA_PID_H

class PID {
public:
    float kp;
    float ki;
    float kd;
    float targetpoint;
    // Last time error
    float prevError;
    // integral
    float intergral;
    // differential
    float derivative;

    PID(float kp, float ki, float kd);

    void Set_PID(float kp, float ki, float kd);

    /**
     * pid calculation function
     * @param target  target value
     * @param current current value
     * @return  output
     */
    float compute(float target, float current);

    /**
     * Reset all errors.
     */
    void reset();
};

#endif // YAHBOOMCAR_ASTRA_PID_H
