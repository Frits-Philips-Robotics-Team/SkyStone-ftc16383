package org.firstinspires.ftc.teamcode.NotOpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FritsBot {
    public HDrive drivetrain = new HDrive();
    public TestSideGrabber testGrabber = new TestSideGrabber();
    public SideGrabber sideGrabber = new SideGrabber();
    private LiftGrab liftGrab = new LiftGrab();
    private Intake fritsIntake = new Intake();
    private BNO055IMU imu;
    private PD_controller rotationPD = new PD_controller();

    // objects for driveHoldAngle
    private boolean wasRotating;

    private double prevAngle;
    private double rotateOffset;

    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        testGrabber.init(hwMap);
        sideGrabber.init(hwMap);
        liftGrab.init(hwMap);
        fritsIntake.init(hwMap);
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        wasRotating = false;
        rotationPD.target = getHeadingRadians();

        prevAngle = 0;
        rotateOffset = 0;
    }

    public void driveSimple (double forward, double strafe, double rotate) {
        drivetrain.drive(forward, strafe, rotate);
    }

//    public void driveFieldCentric(double forward, double strafe, double rotate) {
//        Polar driveP = Polar.fromXYCoord(strafe, forward);
//        double heading = getHeadingRadians();
//
//        driveP.subtractAngle(heading);
//        drivetrain.drive(driveP.getY(), driveP.getX(), rotate);
//    }

    public void driveHoldAngle (double forward, double strafe, double rotate) {
        double heading = getHeadingRadians();
        Polar driveP = Polar.fromXYCoord(strafe, forward);

        if (rotate == 0 && !wasRotating) {
            rotationPD.current = heading;
        } else if (wasRotating && rotate == 0) {
            wasRotating = false;
            rotationPD.target = heading;
        } else if (!wasRotating) {
            wasRotating = true;
        }

        driveP.subtractAngle(heading);

        drivetrain.drive(driveP.getY(), driveP.getX(), rotationPD.getOutput());
    }

    public void setIntakePower (double power) {
        fritsIntake.setIntakePower(power);
    }

    private double getHeadingRadians() {
        Orientation angles;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        if(angles.firstAngle < 0 && prevAngle > 2.8) {
            rotateOffset += 2 * Math.PI;
        }
        else if(angles.firstAngle > 0 && prevAngle < -2.8) {
            rotateOffset -= 2 * Math.PI;
        }

        prevAngle = angles.firstAngle;
        return -angles.firstAngle - rotateOffset;
    }

//    public void reportHeadingRadians(Telemetry telemetry) {
//        telemetry.addData("heading:", getHeadingRadians());
//    }
}