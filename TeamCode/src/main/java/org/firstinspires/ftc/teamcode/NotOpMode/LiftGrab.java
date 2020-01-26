package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.jetbrains.annotations.NotNull;

public class LiftGrab {

    private DcMotor grabberSlide;
    private Servo gripper;
    private DcMotor liftLeft;
    private DcMotor liftRight;

    private ElapsedTime encoderTime = new ElapsedTime();

    public void init(@NotNull HardwareMap hwMap) {
//        grabberSlide = hwMap.get(DcMotor.class, "grabber_slide");
//        gripper = hwMap.get(Servo.class, "lift_gripper");
        liftLeft = hwMap.get(DcMotor.class, "lift_left");
        liftRight = hwMap.get(DcMotor.class, "lift_right");

//        grabberSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        grabberSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveGrabber(@NotNull String armPos, String gripperPos) {
        final int inValue = 0;
        final int outValue = 424;

        final double openValue = 0.34;
        final double closedValue = 0;

        switch (armPos) {
            case "in":
                grabberSlide.setTargetPosition(inValue);
                grabberSlide.setPower(0.7);
                grabberSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case "out":
                grabberSlide.setTargetPosition(outValue);
                grabberSlide.setPower(0.7);
                grabberSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        liftLeft.setPower(0.9 * liftPower);
        liftRight.setPower(0.9 * liftPower);
    }

    public void encoderLift(double speed, double cm, double TimeoutS, @NotNull LinearOpMode opmode) {

        final double LIFT_COUNTS_PER_CM = 4.535; // (288 / 1.333...) / (15 * 0.635 cm * 5)
        int NewLiftTarget;

        if(opmode.opModeIsActive()) {
            NewLiftTarget = liftLeft.getCurrentPosition() + (int) (cm * LIFT_COUNTS_PER_CM);
            liftLeft.setTargetPosition(NewLiftTarget);
            liftRight.setTargetPosition(NewLiftTarget);

            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            encoderTime.reset();
            liftLeft.setPower(Math.abs(speed));
            liftRight.setPower(Math.abs(speed));

            while(opmode.opModeIsActive() && (encoderTime.seconds() < TimeoutS) && liftLeft.isBusy());

            liftLeft.setPower(0);
            liftRight.setPower(0);

            liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
