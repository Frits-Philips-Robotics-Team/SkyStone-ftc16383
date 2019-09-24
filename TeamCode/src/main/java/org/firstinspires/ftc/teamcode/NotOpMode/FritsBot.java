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

    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        fritsIntake.init(hwMap);
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
    }

    public void driveSimple(double forward, double strafe, double rotate) {
        drivetrain.drive(forward, strafe, rotate);
    }

    public void driveFieldCentric(double forward, double strafe, double rotate) {
        Polar driveP = Polar.fromXYCoord(strafe, forward);
        double heading = getHeadingRadians();

        driveP.subtractAngle(heading);
        drivetrain.drive(driveP.getY(), driveP. getX(), rotate);
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
