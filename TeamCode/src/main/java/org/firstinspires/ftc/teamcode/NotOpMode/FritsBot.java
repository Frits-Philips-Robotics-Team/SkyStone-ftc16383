package org.firstinspires.ftc.teamcode.NotOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FritsBot {
    HDrive drivetrain = new HDrive();
    private BNO055IMU imu;

    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
    }

    public void driveSimple(double forward, double strafe, double rotate) {
        drivetrain.drive(forward, strafe, rotate);
    }

    public void driveFieldCentric(double forward, double strafe, double rotate) {

    }
}
