// pid_controller.h
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
class PIDController {
public:
    PIDController(double kp, double ki, double kd);
    double compute(double target, double actual, double dt);
private:
    double _kp, _ki, _kd, _integral = 0, _prev_error = 0;
};
#endif

