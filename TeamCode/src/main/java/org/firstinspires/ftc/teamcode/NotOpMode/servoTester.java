package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class servoTester {
    private Servo servo_one;
    private Servo servo_two;
    private ElapsedTime cycleTimeOne = new ElapsedTime();
    private ElapsedTime cycleTimeTwo = new ElapsedTime();

    private double angleOne;
    private double angleTwo;

    public void init(HardwareMap hwMap) {
        servo_one = hwMap.get(Servo.class, "left_foundation");
        servo_two = hwMap.get(Servo.class, "right_foundation");

        angleOne = 0.5;
        angleTwo = 0.5;

        cycleTimeOne.reset();
        cycleTimeTwo.reset();
    }

    public void moveOne(boolean up) {
        if(cycleTimeOne.milliseconds() >= 50) {
            if(up) {
                angleOne += 0.01;
            }
            else {
                angleOne -= 0.01;
            }
            servo_one.setPosition(angleOne);
        }
    }

    public void moveTwo(boolean up) {
        if(cycleTimeTwo.milliseconds() >= 50) {
            if(up) {
                angleTwo += 0.01;
            }
            else {
                angleTwo -= 0.01;
            }
            servo_two.setPosition(angleTwo);
        }
    }

    public void reportPositions(Telemetry telemetry) {
        telemetry.addData("servos", "servo_one: %f, servo_two: %f", angleOne, angleTwo);
    }
}
