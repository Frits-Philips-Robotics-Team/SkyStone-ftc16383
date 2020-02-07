package org.firstinspires.ftc.teamcode.NotOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FritsBot {

    public XDrive drivetrain = new XDrive();
    public servoTester servoTest = new servoTester();
    public SideGrabber sideGrabber = new SideGrabber();
    public LiftGrab liftGrab = new LiftGrab();
    public FoundationServo foundationServo = new FoundationServo();
    public Intake intake = new Intake();
    private BNO055IMU imu;
    public DistanceSensor rightDistance;

    private ElapsedTime rotateTimer = new ElapsedTime();
    private ElapsedTime autonTime = new ElapsedTime();
    // objects for driveHoldAngle
    private boolean wasRotating;
    private double rotationSetpoint;
    private double holdAngleOffset;

    private double prevAngle;
    private double fullRotationOffset;

    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        servoTest.init(hwMap);
        sideGrabber.init(hwMap);
        liftGrab.init(hwMap);
        foundationServo.init(hwMap);
        intake.init(hwMap);
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        rightDistance = hwMap.get(DistanceSensor.class, "right_distance");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)rightDistance;

        wasRotating = false;
        rotationSetpoint = getHeadingRadians();

        prevAngle = 0;
        fullRotationOffset = 0;
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
        double currentHeading = getHeadingRadians();

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

    public void driveToPoint(double speed, double x, double y, double targetHeading, double timeoutS) {
        Polar autonDrive = Polar.fromXYCoord(-x, -y);
        // Shift distance vector 45 degrees clockwise so we can get diagonal distances from x and y
        autonDrive.subtractAngle(0.7853981633973); // PI/4

        double diagonalRightCM = autonDrive.getX();
        double diagonalLeftCM = autonDrive.getY();
        encoderDrive(speed, diagonalRightCM, diagonalLeftCM, targetHeading, timeoutS);
    }

    // rotate relative to rotation setpoint
    public void rotateAbsolute(double speed, int rotation, double timeoutS) {
        double currentHeading = getHeadingRadians();
        int wasAtSetpoint = 0;

        double kP = 0.5;
        double kI = 0.0018;
        double kD = 20;
        long dt = 20;
        double integral = 0;
        double derivative = 0;
        double prevError = 0;

        rotationSetpoint = Math.toRadians(rotation) - holdAngleOffset;

        double error;
        rotateTimer.reset();

        while (rotateTimer.seconds() < timeoutS && wasAtSetpoint != 4 && opmode.opModeIsActive()) {
            currentHeading = getHeadingRadians();
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

    private void encoderDrive(double speed, double diagonalRightCM, double diagonalLeftCM, double headingTarget, double TimeoutS) {

        final double COUNTS_PER_CM = 22.765458802; // (1120 / 2) / (9 * 3.1415)
        final double gain = 0; //TODO: find a good heading gain value

        int newFlTarget;
        int newRlTarget;
        int newRrTarget;
        int newFrTarget;

        double headingSetpoint = Math.toRadians(headingTarget);

        // Determine new target position, and pass to motor controller
        newFlTarget = drivetrain.flDrive.getCurrentPosition() - (int) (diagonalRightCM * COUNTS_PER_CM);
        newRlTarget = drivetrain.rlDrive.getCurrentPosition() - (int) (diagonalLeftCM * COUNTS_PER_CM);
        newRrTarget = drivetrain.rrDrive.getCurrentPosition() - (int) (diagonalRightCM * COUNTS_PER_CM);
        newFrTarget = drivetrain.frDrive.getCurrentPosition() - (int) (diagonalLeftCM * COUNTS_PER_CM);
        drivetrain.flDrive.setTargetPosition(newFlTarget);
        drivetrain.rlDrive.setTargetPosition(newRlTarget);
        drivetrain.rrDrive.setTargetPosition(newRrTarget);
        drivetrain.frDrive.setTargetPosition(newFrTarget);

        // Turn On RUN_TO_POSITION
        drivetrain.flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivetrain.rlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivetrain.rrDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivetrain.frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double diagonalLeftSpeed;
        double diagonalRightSpeed;
        if (Math.abs(diagonalLeftCM / diagonalRightCM) < 1) {
            diagonalLeftSpeed = (diagonalLeftCM / diagonalRightCM) * speed;
            diagonalRightSpeed = speed;
        }
        else {
            diagonalLeftSpeed = speed;
            diagonalRightSpeed = (diagonalRightCM / diagonalLeftCM) * speed;
        }

        // reset the timeout time and start motion.
        autonTime.reset();

        while(opmode.opModeIsActive() && (autonTime.seconds() < TimeoutS) && (drivetrain.flDrive.isBusy() || drivetrain.rlDrive.isBusy() || drivetrain.rrDrive.isBusy())) {
            // wait until motors are done
            double error = headingSetpoint - getHeadingRadians();
            double steering = error * gain;
            drivetrain.flDrive.setPower(Math.abs(diagonalRightSpeed) + steering);
            drivetrain.rlDrive.setPower(Math.abs(diagonalLeftSpeed) + steering);
            drivetrain.rrDrive.setPower(Math.abs(diagonalRightSpeed) - steering);
            drivetrain.frDrive.setPower(Math.abs(diagonalLeftSpeed) - steering);
        }

        // Stop all motion;
        drivetrain.flDrive.setPower(0);
        drivetrain.rlDrive.setPower(0);
        drivetrain.rrDrive.setPower(0);
        drivetrain.frDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        drivetrain.flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrain.rlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrain.rrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrain.frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetDriveAngle() {
        holdAngleOffset = getHeadingRadians();
    }

    private double getHeadingRadians() {
        Orientation angles;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        if(angles.firstAngle < 0 && prevAngle > 2.8) {
            fullRotationOffset += 2 * Math.PI;
        }
        else if(angles.firstAngle > 0 && prevAngle < -2.8) {
            fullRotationOffset -= 2 * Math.PI;
        }

        prevAngle = angles.firstAngle;
        return -angles.firstAngle - fullRotationOffset;
    }

    public void reportHeadingRadians(Telemetry telemetry) {
        telemetry.addData("heading:", getHeadingRadians());
    }
}