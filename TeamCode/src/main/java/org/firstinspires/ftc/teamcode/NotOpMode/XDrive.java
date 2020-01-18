package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;

public class XDrive {

    private DcMotor flDrive;
    private DcMotor rlDrive;
    private DcMotor rrDrive;
    private DcMotor frDrive;
    private ElapsedTime cycleTime = new ElapsedTime();
    private ElapsedTime autonTime = new ElapsedTime();

    private static final int CYCLE_MS = 40;
    private double maxSpeed;
    private double increment;

    private double flPowerCurrent;
    private double rlPowerCurrent;
    private double rrPowerCurrent;
    private double frPowerCurrent;

    void init(HardwareMap hwMap) {
        flDrive = hwMap.get(DcMotor.class, "FL_drive");
        rlDrive = hwMap.get(DcMotor.class, "RL_drive");
        rrDrive = hwMap.get(DcMotor.class, "RR_drive");
        frDrive = hwMap.get(DcMotor.class, "FR_drive");

        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flDrive.setDirection(DcMotor.Direction.FORWARD);
        rlDrive.setDirection(DcMotor.Direction.FORWARD);
        rrDrive.setDirection(DcMotor.Direction.REVERSE);
        frDrive.setDirection(DcMotor.Direction.REVERSE);

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rlDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        maxSpeed = 1;
        increment = 0.06;
        flPowerCurrent = 0;
        rlPowerCurrent = 0;
        rrPowerCurrent = 0;
    }

    private LinearOpMode opmode;

    void initAutonomous(LinearOpMode opmode) {
        this.opmode = opmode;
    }

    public void reportEncoders(Telemetry telemetry) {
        telemetry.addData("encoders", "%d, %d, %d, %d",
                flDrive.getCurrentPosition(),
                rlDrive.getCurrentPosition(),
                rrDrive.getCurrentPosition(),
                frDrive.getCurrentPosition());
    }

    void directDrive(double forwardSpeed, double strafeSpeed, double rotateSpeed) {
        double flDriveSpeed = Range.clip(forwardSpeed + strafeSpeed + rotateSpeed, -maxSpeed, maxSpeed);
        double rlDriveSpeed = Range.clip(forwardSpeed - strafeSpeed + rotateSpeed, -maxSpeed, maxSpeed);
        double rrDriveSpeed = Range.clip(forwardSpeed + strafeSpeed - rotateSpeed, -maxSpeed, maxSpeed);
        double frDriveSpeed = Range.clip(forwardSpeed - strafeSpeed - rotateSpeed, -maxSpeed, maxSpeed);

        flDrive.setPower(flDriveSpeed);
        rlDrive.setPower(rlDriveSpeed);
        rrDrive.setPower(rrDriveSpeed);
        frDrive.setPower(frDriveSpeed);
    }

    void drive(double forwardSpeed, double strafeSpeed, double rotateSpeed) {
        double flDriveSpeed = Range.clip(forwardSpeed + strafeSpeed + rotateSpeed, -maxSpeed, maxSpeed);
        double rlDriveSpeed = Range.clip(forwardSpeed - strafeSpeed + rotateSpeed, -maxSpeed, maxSpeed);
        double rrDriveSpeed = Range.clip(forwardSpeed + strafeSpeed - rotateSpeed, -maxSpeed, maxSpeed);
        double frDriveSpeed = Range.clip(forwardSpeed - strafeSpeed - rotateSpeed, -maxSpeed, maxSpeed);

        accelToSpeed(flDriveSpeed, rlDriveSpeed, rrDriveSpeed, frDriveSpeed);
    }

    private void accelToSpeed(double flDriveSpeed, double rlDriveSpeed, double rrDriveSpeed, double frDriveSpeed) {
        if (cycleTime.milliseconds() > CYCLE_MS) {

            if(flDriveSpeed == 0) {
                flPowerCurrent = 0;
                flDrive.setPower(0);
            } else if (flDriveSpeed > flPowerCurrent) {
                flPowerCurrent = Range.clip(flPowerCurrent + increment, -1, flDriveSpeed);
                flDrive.setPower(maxSpeed * flPowerCurrent);
            } else if (flDriveSpeed < flPowerCurrent) {
                flPowerCurrent = Range.clip(flPowerCurrent - increment, flDriveSpeed, 1);
                flDrive.setPower(maxSpeed * flPowerCurrent);
            } else {
                flDrive.setPower(maxSpeed * flPowerCurrent);
            }

            if(rlDriveSpeed == 0) {
                rlPowerCurrent = 0;
                rlDrive.setPower(0);
            } else if (rlDriveSpeed > rlPowerCurrent) {
                rlPowerCurrent = Range.clip(rlPowerCurrent + increment, -1, rlDriveSpeed);
                rlDrive.setPower(maxSpeed * rlPowerCurrent);
            } else if (rlDriveSpeed < rlPowerCurrent) {
                rlPowerCurrent = Range.clip(rlPowerCurrent - increment, rlDriveSpeed, 1);
                rlDrive.setPower(maxSpeed * rlPowerCurrent);
            } else {
                rlDrive.setPower(maxSpeed * rlPowerCurrent);
            }

            if(rrDriveSpeed == 0) {
                rrPowerCurrent = 0;
                rrDrive.setPower(0);
            } else if (rrDriveSpeed > rrPowerCurrent) {
                rrPowerCurrent = Range.clip(rrPowerCurrent + increment, -1, rrDriveSpeed);
                rrDrive.setPower(maxSpeed * rrPowerCurrent);
            } else if (rrDriveSpeed < rrPowerCurrent) {
                rrPowerCurrent = Range.clip(rrPowerCurrent - increment, rrDriveSpeed, 1);
                rrDrive.setPower(maxSpeed * rrPowerCurrent);
            } else {
                rrDrive.setPower(maxSpeed * rrPowerCurrent);
            }

            if(frDriveSpeed == 0) {
                frPowerCurrent = 0;
                frDrive.setPower(0);
            } else if (frDriveSpeed > frPowerCurrent) {
                frPowerCurrent = Range.clip(frPowerCurrent + increment, -1, frDriveSpeed);
                frDrive.setPower(maxSpeed * frPowerCurrent);
            } else if (frDriveSpeed < frPowerCurrent) {
                frPowerCurrent = Range.clip(frPowerCurrent - increment, frDriveSpeed, 1);
                frDrive.setPower(maxSpeed * frPowerCurrent);
            } else {
                frDrive.setPower(maxSpeed * frPowerCurrent);
            }
        }
    }

    public void setSpeed(double value) {
        maxSpeed = value;
    }

    public void encoderDrive(double speed, double diagonalRightCM, double diagonalLeftCM, double TimeoutS) {

        final double COUNTS_PER_CM = 19.8059; // (1120 / 2) / (9 * 3.1415)

        int newFlTarget;
        int newRlTarget;
        int newRrTarget;
        int newFrTarget;

        // Determine new target position, and pass to motor controller
        newFlTarget = flDrive.getCurrentPosition() - (int) (diagonalRightCM * COUNTS_PER_CM);
        newRlTarget = rlDrive.getCurrentPosition() - (int) (diagonalLeftCM * COUNTS_PER_CM);
        newRrTarget = rlDrive.getCurrentPosition() - (int) (diagonalRightCM * COUNTS_PER_CM);
        newFrTarget = rlDrive.getCurrentPosition() - (int) (diagonalLeftCM * COUNTS_PER_CM);
        flDrive.setTargetPosition(newFlTarget);
        rlDrive.setTargetPosition(newRlTarget);
        rrDrive.setTargetPosition(newRrTarget);
        frDrive.setTargetPosition(newFrTarget);

        // Turn On RUN_TO_POSITION
        flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        autonTime.reset();
        flDrive.setPower(Math.abs(speed));
        rlDrive.setPower(Math.abs(speed));
        rrDrive.setPower(Math.abs(speed));
        frDrive.setPower(Math.abs(speed));

        while(opmode.opModeIsActive() && (autonTime.seconds() < TimeoutS) && (flDrive.isBusy() || rlDrive.isBusy() || rrDrive.isBusy())) {
            // wait until motors are done
        }

        // Stop all motion;
        flDrive.setPower(0);
        rlDrive.setPower(0);
        rrDrive.setPower(0);
        frDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void reportSpeeds(Telemetry telemetry) {
        telemetry.addData("FL_speed: ", flPowerCurrent);
        telemetry.addData("RL_speed: ", rlPowerCurrent);
        telemetry.addData("RR_speed: ", rrPowerCurrent);
        telemetry.addData("FR_speed: ", frPowerCurrent);
    }
}
