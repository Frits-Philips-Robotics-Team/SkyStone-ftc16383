package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SideGrabber {

    // TODO: find out which values the servos need for these positions
    private Servo arm;
    private Servo gripper;
    private ElapsedTime cycleTime = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        arm = hwMap.get(Servo.class, "side_arm");
        gripper = hwMap.get(Servo.class, "side_gripper");

        cycleTime.reset();
    }

    public void moveGrabber(String armPos, String gripperPos) {
        final double upValue = 1;
        final double downValue = 0;
        final double openValue = 1;
        final double closedValue = 0.07;
        final double initialValue = 1;

        switch (armPos) {
            case "up":      arm.setPosition(upValue);
            break;
            case "down":    arm.setPosition(downValue);
            break;
        }
        switch (gripperPos) {
            case "open":    gripper.setPosition(openValue);
            break;
            case "closed":  gripper.setPosition(closedValue);
            break;
            case "initial": gripper.setPosition(initialValue);
            break;
        }
    }
}
