package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SideGrabber {

    private Servo leftArm;
    private Servo leftGripper;
    private Servo rightArm;
    private Servo rightGripper;

    public void init(HardwareMap hwMap) {
        leftArm = hwMap.get(Servo.class, "left_grabber_arm");
        leftGripper = hwMap.get(Servo.class, "left_grabber_gripper");
        rightArm = hwMap.get(Servo.class, "right_grabber_arm");
        rightGripper = hwMap.get(Servo.class, "right_grabber_gripper");

        moveLeftGrabber("upEmpty", "initial");
        moveRightGrabber("upEmpty", "initial");
    }

    /*
        right arm: down = 0.19
        up_block = 0.35
        up_max = 0.53

        right leftGripper: max_closed = 0.17
        straight = 0.77
        closed_block = 0.33

        left arm: down = 0.81
        up_block = 0.58
        up_max = 0.49

        left leftGripper: closed_max = 1
        closed_block = 0.85
        straight = 0.45
        */

    public void moveLeftGrabber(String armPos, String gripperPos) {
        final double upBlockValue = 0.58;
        final double upEmptyValue = 0.49;
        final double downValue = 0.81;
        final double openValue = 0.45;
        final double closedValue = 0.85;
        final double initialGripperValue = 1;

        switch (armPos) {
            case "upBlock": leftArm.setPosition(upBlockValue);
            break;
            case "upEmpty": leftArm.setPosition(upEmptyValue);
            break;
            case "down":    leftArm.setPosition(downValue);
            break;
            default: break;
        }
        switch (gripperPos) {
            case "open":    leftGripper.setPosition(openValue);
            break;
            case "closed":  leftGripper.setPosition(closedValue);
            break;
            case "initial": leftGripper.setPosition(initialGripperValue);
            break;
            default: break;
        }
    }

    public void moveRightGrabber(String armPos, String gripperPos) {
        final double upBlockValue = 0.35;
        final double upEmptyValue = 0.53;
        final double downValue = 0.19;
        final double openValue = 0.77;
        final double closedValue = 0.33;
        final double initialGripperValue = 0.17;

        switch (armPos) {
            case "upBlock": rightArm.setPosition(upBlockValue);
                break;
            case "upEmpty": rightArm.setPosition(upEmptyValue);
                break;
            case "down":    rightArm.setPosition(downValue);
                break;
            default: break;
        }
        switch (gripperPos) {
            case "open":    rightGripper.setPosition(openValue);
                break;
            case "closed":  rightGripper.setPosition(closedValue);
                break;
            case "initial": rightGripper.setPosition(initialGripperValue);
                break;
            default: break;
        }
    }
}
