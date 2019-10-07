package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestSideGrabber {
    private Servo arm;
    private Servo gripper;
    private ElapsedTime armCycleTime = new ElapsedTime();
    private ElapsedTime gripperCycleTime = new ElapsedTime();

    private double armAngle;
    private double gripperAngle;

    public void init(HardwareMap hwMap) {
        arm = hwMap.get(Servo.class, "side_arm");
        gripper = hwMap.get(Servo.class, "side_gripper");

        armAngle = 0.5;
        gripperAngle = 0.5;

        armCycleTime.reset();
        gripperCycleTime.reset();
    }

    public void moveArm(boolean up) {
        if(armCycleTime.milliseconds() >= 50) {
            if(up) {
                armAngle += 0.01;
            }
            else {
                armAngle -= 0.01;
            }
            arm.setPosition(armAngle);
        }
    }

    public void moveGripper(boolean up) {
        if(gripperCycleTime.milliseconds() >= 50) {
            if(up) {
                gripperAngle += 0.01;
            }
            else {
                gripperAngle -= 0.01;
            }
            gripper.setPosition(gripperAngle);
        }
    }

    public void reportPositions(Telemetry telemetry) {
        telemetry.addData("servos", "arm: %f, gripper: %f", armAngle, gripperAngle);
    }
}
