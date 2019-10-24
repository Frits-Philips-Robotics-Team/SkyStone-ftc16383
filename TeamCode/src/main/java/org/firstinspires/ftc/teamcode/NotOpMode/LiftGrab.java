package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftGrab {

    private Servo rotateArm;
    private Servo gripper;
    private DcMotor liftOne;
    private DcMotor liftTwo;

    private ElapsedTime cycleTime = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        rotateArm = hwMap.get(Servo.class, "lift_arm");
        gripper = hwMap.get(Servo.class, "lift_gripper");
        liftOne = hwMap.get(DcMotor.class, "liftOne");
        liftTwo = hwMap.get(DcMotor.class, "liftTwo");

        liftOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //moveGrabber("in", "initial");
        cycleTime.reset();
    }

    public void moveGrabber(String armPos, String gripperPos) {
        final double inValue = 1;
        final double outValue = 0;

        final double openValue = 0.58;
        final double closedValue = 0.38;
        final double initialGripperValue = 0.17;

        switch (armPos) {
            case "in":
                rotateArm.setPosition(inValue);
                break;
            case "out":
                rotateArm.setPosition(outValue);
                break;
        }
        switch (gripperPos) {
            case "open":
                gripper.setPosition(openValue);
                break;
            case "closed":
                gripper.setPosition(closedValue);
                break;
            case "initial":
                gripper.setPosition(initialGripperValue);
                break;
        }
    }

    public void setLiftPower(double liftPower) {
        liftOne.setPower(liftPower);
        liftTwo.setPower(liftPower);
    }
}
