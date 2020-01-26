package org.firstinspires.ftc.teamcode.NotOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMU {
    private static BNO055IMU imu;

    private static double prevAngle;
    private static double fullRotationOffset;

    static void init(HardwareMap hwMap) {
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        prevAngle = 0;
        fullRotationOffset = 0;
    }

    double getHeadingRadians() {
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
}
