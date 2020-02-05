package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

public class LiftGrab {

    private DcMotor grabberSlide;
    private Servo gripper;
    private DcMotor liftLeft;
    private DcMotor liftRight;

    private ElapsedTime encoderTime = new ElapsedTime();

    private final String[] levelOptions = {"bottom", "level 0", "level 1", "level 2", "level 3", "level 4", "top"};
    private final int[] encoderValues = {0, 6000};
    private Boolean usingSetHeight;

    public void init(@NotNull HardwareMap hwMap) {
        grabberSlide = hwMap.get(DcMotor.class, "grabber_slide");
        gripper = hwMap.get(Servo.class, "main_grabber_gripper");
        liftLeft = hwMap.get(DcMotor.class, "lift_left");
        liftRight = hwMap.get(DcMotor.class, "lift_right");

//        grabberSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        grabberSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        usingSetHeight = false;

        grabberSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void slideArm(double power) {
        grabberSlide.setPower(power);
    }
    public void moveGrabber(String armPos, String gripperPos) {
        final int inValue = 0;
        final int outValue = 424;

        // max_open = 0.4
        // max_closed = 0.11
        // closed_block = 0.15
        // open_normal = 0.25
        final double openValue = 0.25;
        final double closedValue = 0.15;

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
        if(usingSetHeight) {
            usingSetHeight = false;
            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        double maxPower;
        if (liftLeft.getCurrentPosition() >= encoderValues[encoderValues.length -1]) {
            maxPower = 0;
        }
        else {
            maxPower = 1;
        }

        liftLeft.setPower(Range.clip(0.9 * liftPower, -1, maxPower));
        liftRight.setPower(Range.clip(0.9 * liftPower, -1, maxPower));
    }

    public void setLiftHeight(int blockLevel) {

        liftLeft.setTargetPosition(encoderValues[blockLevel]);
        liftRight.setTargetPosition(encoderValues[blockLevel]);

        if(!usingSetHeight) {
            usingSetHeight = true;
            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
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

    public void reportEncoders(@NotNull Telemetry telemetry) {
        telemetry.addData("leftLift: ", liftLeft.getCurrentPosition());
        telemetry.addData("rightLift: ", liftRight.getCurrentPosition());
        telemetry.addData("grabber slide", grabberSlide.getCurrentPosition());
        telemetry.update();
    }
}
