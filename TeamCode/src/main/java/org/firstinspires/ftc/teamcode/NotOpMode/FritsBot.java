package org.firstinspires.ftc.teamcode.NotOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FritsBot {
    public HDrive drivetrain = new HDrive();
    private Intake fritsIntake = new Intake();
    private BNO055IMU imu;

    // objects for driveHoldAngle
    private boolean wasRotating;
    private double rotation;


    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        fritsIntake.init(hwMap);
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        wasRotating = false;
        rotation = getHeadingRadians();
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
        double heading = getHeadingRadians();

        double rotateNew;
        final double adjustmentSpeed = 0.4;

        if(rotate == 0 && !wasRotating) {
            rotateNew = adjustmentSpeed * (rotation - heading);
        }
        else if(wasRotating && rotate == 0) {
            wasRotating = false;
            rotation = heading;
            rotateNew = 0;
        }
        else if(!wasRotating) {
            wasRotating = true;
            rotateNew = rotate;
        }
        else {
            rotateNew = rotate;
        }

        if(Math.abs(rotateNew) < 0.05) {
            rotateNew = 0;
        }

        driveP.subtractAngle(heading);
        drivetrain.drive(driveP.getY(), driveP.getX(), rotateNew);
    }

    public void setIntakePower(double power) {
        fritsIntake.setIntakePower(power);
    }

    private double getHeadingRadians() {
        Orientation angles;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return -angles.firstAngle;
    }
}