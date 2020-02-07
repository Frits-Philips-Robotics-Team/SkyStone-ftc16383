package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NotOpMode.FritsBot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name= "RedFoundation", group="red")
//@Disabled//comment out this line before using
public class AutonRedFoundation extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private FritsBot robot = new FritsBot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.initAutonomous(this);

        waitForStart();
        runtime.reset();

        robot.driveToPoint(1, -20, -75, 0, 3);
        robot.foundationServo.moveUp(false);
        sleep(1000);
        robot.driveToPoint(1, 0, 85, 0, 3);
        robot.rotateAbsolute(1, 0, 2);
        robot.driveToPoint(0.7, 0, 20, 0, 2);
        robot.foundationServo.moveUp(true);
        sleep(500);
        robot.driveToPoint(0.4, 0, 5, 0, 1);
        robot.driveToPoint(0.8, 120, 0, 0, 3);
        robot.driveToPoint(0.7, 0, 20, 0, 1);
    }
}