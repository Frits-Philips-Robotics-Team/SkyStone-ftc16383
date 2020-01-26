package org.firstinspires.ftc.teamcode.NotOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FritsBot {

    public XDrive drivetrain = new XDrive();
    //public servoTester servoTest = new servoTester();
    public SideGrabber sideGrabber = new SideGrabber();
    public LiftGrab liftGrab = new LiftGrab();
    public FoundationServo foundationServo = new FoundationServo();
    public Intake intake = new Intake();
    private IMU imu;

    private ElapsedTime rotateTimer = new ElapsedTime();
    // objects for driveHoldAngle
    private boolean wasRotating;
    private double rotationSetpoint;
    private double holdAngleOffset;

    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        //servoTest.init(hwMap);
//        sideGrabber.init(hwMap);
        liftGrab.init(hwMap);
//        foundationServo.init(hwMap);
        intake.init(hwMap);
        imu.init(hwMap);

        wasRotating = false;
        rotationSetpoint = imu.getHeadingRadians();

        holdAngleOffset = 0;
    }

    private LinearOpMode opmode;

    public void initAutonomous(LinearOpMode opmode) {
        this.opmode = opmode;
        drivetrain.initAutonomous(opmode);
    }

    public void driveSimple(double forward, double strafe, double rotate) {
        drivetrain.drive(forward, strafe, rotate);
    }

//    public void driveFieldCentric(double forward, double strafe, double rotate) {
//        Polar driveP = Polar.fromXYCoord(strafe, forward);
//        double heading = getHeadingRadians();
//
//        driveP.subtractAngle(heading);
//        drivetrain.drive(driveP.getY(), driveP.getX(), rotate);
//    }

    public void driveHoldAngle(double forward, double strafe, double rotate) {
        Polar driveP = Polar.fromXYCoord(strafe, forward);
        double currentHeading = imu.getHeadingRadians();

        double rotateNew;
        final double adjustmentSpeed = 0.4;


        if (rotate == 0 && !wasRotating) {
            rotateNew = adjustmentSpeed * (rotationSetpoint - currentHeading);
        } else if (wasRotating && rotate == 0) {
            wasRotating = false;
            rotationSetpoint = currentHeading;
            rotateNew = 0;
        } else if (!wasRotating) {
            wasRotating = true;
            rotateNew = rotate;
            rotateTimer.reset();
        } else {
            rotateNew = rotate;
            rotateTimer.reset();
        }

        if (rotateTimer.milliseconds() < 300) {
            rotationSetpoint = currentHeading;
        }

        if (Math.abs(rotateNew) < 0.02) {
            rotateNew = 0;
        }

        driveP.subtractAngle(-currentHeading + holdAngleOffset);
        drivetrain.drive(driveP.getY(), driveP.getX(), rotateNew);
    }

    public void driveToPoint(double speed, double x, double y, double headingTarget, double timeoutS) {
        Polar autonDrive = Polar.fromXYCoord(x, y);
        // Shift distance vector 45 degrees clockwise so we can get diagonal distances from x and y
        autonDrive.subtractAngle(0.7853981633973); // PI/4

        double diagonalRightCM = autonDrive.getX();
        double diagonalLeftCM = autonDrive.getY();
        drivetrain.encoderDrive(speed, diagonalRightCM, diagonalLeftCM, headingTarget, timeoutS);
    }

    // rotate relative to rotation setpoint
    public void rotateAbsolute(double speed, int rotation, double timeoutS, Telemetry telemetry) {
        double currentHeading = imu.getHeadingRadians();
        int wasAtSetpoint = 0;

        double kP = 0.8;
        double kI = 0;
        double kD = 0.4;
        long dt = 20;
        double integral = 0;
        double derivative = 0;
        double prevError = 0;

        rotationSetpoint = Math.toRadians(rotation) - holdAngleOffset;

        double error;
        rotateTimer.reset();

        while (rotateTimer.seconds() < timeoutS && wasAtSetpoint != 4 && opmode.opModeIsActive()) {
            currentHeading = imu.getHeadingRadians();
            error = rotationSetpoint - currentHeading;
            integral = integral + error * dt;
            derivative = (error - prevError) / dt;

            double rotateSpeed = Range.clip(speed * (kP * error + kI * integral + kD * derivative), -1, 1);
            drivetrain.directDrive(0, 0, rotateSpeed);
            prevError = error;

            if (Math.abs(error) < 0.02) {
                wasAtSetpoint++;
            }

//            telemetry.addData("target: ", rotationSetpoint);
//            reportHeadingRadians(telemetry);
//            telemetry.addData("error: ", error);
//            telemetry.update();
            opmode.sleep(dt);
        }
    }

    public void resetDriveAngle() {
        holdAngleOffset = imu.getHeadingRadians();
    }

    public void reportHeadingRadians(Telemetry telemetry) {
        telemetry.addData("heading:", imu.getHeadingRadians());
    }
}