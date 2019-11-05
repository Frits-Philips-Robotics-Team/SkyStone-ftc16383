package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import static java.lang.Thread.sleep;

public class HDrive {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor midDrive;
    private ElapsedTime cycleTime = new ElapsedTime();
    private ElapsedTime autonTime = new ElapsedTime();

    private static final int CYCLE_MS = 40;
    private double maxSpeed;
    private double increment;

    private double leftPowerCurrent;
    private double rightPowerCurrent;
    private double midPowerCurrent;

    void init(HardwareMap hwMap) {
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        midDrive = hwMap.get(DcMotor.class, "mid_drive");

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        midDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        midDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        midDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        maxSpeed = 1;
        increment = 0.06;
        leftPowerCurrent = 0;
        rightPowerCurrent = 0;
        midPowerCurrent = 0;
    }

    public void reportEncoders(Telemetry telemetry) {
        telemetry.addData("encoders", "%d, %d, %d, %d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition(),
                midDrive.getCurrentPosition());
    }

    void drive(double forwardSpeed, double strafeSpeed, double rotateSpeed) {
        double LDriveSpeed = Range.clip(forwardSpeed - rotateSpeed, -maxSpeed, maxSpeed);
        double RDriveSpeed = Range.clip(forwardSpeed + rotateSpeed, -maxSpeed, maxSpeed);
        double midDriveSpeed = Range.clip(strafeSpeed, -maxSpeed, maxSpeed);

        accelToSpeed(LDriveSpeed, RDriveSpeed, midDriveSpeed);
    }

    private void accelToSpeed(double LDriveSpeed, double RDriveSpeed, double midDriveSpeed) {
        if (cycleTime.milliseconds() > CYCLE_MS) {

            if(LDriveSpeed == 0) {
                leftPowerCurrent = 0;
                leftDrive.setPower(0);
            } else if (LDriveSpeed > leftPowerCurrent) {
                leftPowerCurrent = Range.clip(leftPowerCurrent + increment, -1, LDriveSpeed);
                leftDrive.setPower(maxSpeed * leftPowerCurrent);
            } else if (LDriveSpeed < leftPowerCurrent) {
                leftPowerCurrent = Range.clip(leftPowerCurrent - increment, LDriveSpeed, 1);
                leftDrive.setPower(maxSpeed * leftPowerCurrent);
            } else {
                leftDrive.setPower(maxSpeed * leftPowerCurrent);
            }

            if(RDriveSpeed == 0) {
                rightPowerCurrent = 0;
                rightDrive.setPower(0);
            } else if (RDriveSpeed > rightPowerCurrent) {
                rightPowerCurrent = Range.clip(rightPowerCurrent + increment, -1, RDriveSpeed);
                rightDrive.setPower(maxSpeed * rightPowerCurrent);
            } else if (RDriveSpeed < rightPowerCurrent) {
                rightPowerCurrent = Range.clip(rightPowerCurrent - increment, RDriveSpeed, 1);
                rightDrive.setPower(maxSpeed * rightPowerCurrent);
            } else {
                rightDrive.setPower(maxSpeed * rightPowerCurrent);
            }

            if(midDriveSpeed == 0) {
                midPowerCurrent = 0;
                midDrive.setPower(0);
            } else if (midDriveSpeed > midPowerCurrent) {
                midPowerCurrent = Range.clip(midPowerCurrent + increment, -1, midDriveSpeed);
                midDrive.setPower(maxSpeed * midPowerCurrent);
            } else if (midDriveSpeed < midPowerCurrent) {
                midPowerCurrent = Range.clip(midPowerCurrent - increment, midDriveSpeed, 1);
                midDrive.setPower(maxSpeed * midPowerCurrent);
            } else {
                midDrive.setPower(maxSpeed * midPowerCurrent);
            }
        }
    }

    public void setSpeed(double value) {
        maxSpeed = value;
    }

    public void encoderDrive(double leftSpeed, double rightSpeed, double midSpeed,
                             double leftCM, double rightCM, double midCM,
                             double TimeoutS, LinearOpMode opmode) {

        final double COUNTS_PER_CM = 19.8059; // (1120 / 2) / (9 * 3.1415)

        int newLeftTarget;
        int newRightTarget;
        int newMidTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.getCurrentPosition() - (int) (leftCM * COUNTS_PER_CM);
        newRightTarget = rightDrive.getCurrentPosition() - (int) (rightCM * COUNTS_PER_CM);
        newMidTarget = midDrive.getCurrentPosition() - (int) (midCM * COUNTS_PER_CM);
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        midDrive.setTargetPosition(newMidTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        midDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        autonTime.reset();
        leftDrive.setPower(Math.abs(leftSpeed));
        rightDrive.setPower(Math.abs(rightSpeed));
        midDrive.setPower(Math.abs(midSpeed));

        while(opmode.opModeIsActive() && (autonTime.seconds() < TimeoutS) && (leftDrive.isBusy() || rightDrive.isBusy() || midDrive.isBusy())) {
            // wait until motors are done
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        midDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        midDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
