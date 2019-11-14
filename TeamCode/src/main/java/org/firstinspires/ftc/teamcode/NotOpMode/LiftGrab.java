package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftGrab {

    private Servo rotateArm;
    private Servo gripper;
    private DcMotor liftOne;
    private DcMotor liftTwo;

    private ElapsedTime encoderTime = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        rotateArm = hwMap.get(Servo.class, "lift_arm");
        gripper = hwMap.get(Servo.class, "lift_gripper");
        liftOne = hwMap.get(DcMotor.class, "liftOne");
        liftTwo = hwMap.get(DcMotor.class, "liftTwo");

        liftOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveGrabber(String armPos, String gripperPos) {
        final double inValue = 0.74;
        final double outValue = 0;

        final double openValue = 0.34;
        final double closedValue = 0;

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
        }
    }

    public void setLiftPower(double liftPower) {
        liftOne.setPower(liftPower);
        liftTwo.setPower(liftPower);
    }

    public void encoderLift(double speed, double cm, double TimeoutS, LinearOpMode opmode) {

        final double LIFT_COUNTS_PER_CM = 4.535; // (288 / 1.333...) / (15 * 0.635 cm * 5)
        int NewLiftTarget;

        if(opmode.opModeIsActive()) {
            NewLiftTarget = liftOne.getCurrentPosition() + (int) (cm * LIFT_COUNTS_PER_CM);
            liftOne.setTargetPosition(NewLiftTarget);
            liftTwo.setTargetPosition(NewLiftTarget);

            liftOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            encoderTime.reset();
            liftOne.setPower(Math.abs(speed));
            liftTwo.setPower(Math.abs(speed));

            while(opmode.opModeIsActive() && (encoderTime.seconds() < TimeoutS) && liftOne.isBusy());

            liftOne.setPower(0);
            liftTwo.setPower(0);

            liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
