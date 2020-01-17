package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SideGrabber {

    private Servo arm;
    private Servo gripper;

    public void init(HardwareMap hwMap) {
//        arm = hwMap.get(Servo.class, "side_arm");
//        gripper = hwMap.get(Servo.class, "side_gripper");

        moveGrabber("upEmpty", "initial");
    }

    public void moveGrabber(String armPos, String gripperPos) {
        final double upBlockValue = 0.38;
        final double upEmptyValue = 0.41;
        final double downValue = 0.82;
        final double openValue = 0.34;
        final double closedValue = 0.8;
        final double initialGripperValue = 0.91;

        switch (armPos) {
            case "upBlock": arm.setPosition(upBlockValue);
            break;
            case "upEmpty": arm.setPosition(upEmptyValue);
            break;
            case "down":    arm.setPosition(downValue);
            break;
            default: break;
        }
        switch (gripperPos) {
            case "open":    gripper.setPosition(openValue);
            break;
            case "closed":  gripper.setPosition(closedValue);
            break;
            case "initial": gripper.setPosition(initialGripperValue);
            break;
            default: break;
        }
    }
}
