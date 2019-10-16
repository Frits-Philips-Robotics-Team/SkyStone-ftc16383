package org.firstinspires.ftc.teamcode.NotOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;

class PD_controller {
    double target;
    double current;
    private double prevError;

    private ElapsedTime controlTime = new ElapsedTime();


    double getOutput() {
        final double Kp = 0.1;
        final double Kd = 0.2;
        final double minimumOutput = 0.02;

        double dt = controlTime.milliseconds();

        double error = target - current;
        double derivative = (error - prevError) / dt;

        prevError = error;
        controlTime.reset();

        double output = Kp * error + Kd * derivative;

        if (output < minimumOutput) {
            output = 0;
        }

        return output;
    }
}
