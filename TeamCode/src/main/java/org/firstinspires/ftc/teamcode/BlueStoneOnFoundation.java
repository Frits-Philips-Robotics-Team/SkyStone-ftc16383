package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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


@Autonomous(name= "BlueStoneOnFound", group="blue")
//@Disabled//comment out this line before using
public class BlueStoneOnFoundation extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();


    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;
    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 1f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {3.6f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6.2f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    private OpenCvCamera phoneCam;

    private FritsBot robot = new FritsBot();

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT);//display on RC
        //width, height
        //width = height if camera is in portrait mode

        robot.init(hardwareMap);
        robot.initAutonomous(this);

        waitForStart();
        runtime.reset();

        telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
        telemetry.addData("Height", rows);
        telemetry.addData("Width", cols);
        telemetry.update();
        if (valLeft == 0) { // skystone is in position 1 and 4
            robot.driveToPoint(0.8, -70, 5, 0, 2);
            robot.sideGrabber.moveRightGrabber("upEmpty", "open");
            sleep(400);
            robot.sideGrabber.moveRightGrabber("down", "open");
            sleep(400);
            robot.sideGrabber.moveRightGrabber("down", "closed");
            sleep(400);
            robot.sideGrabber.moveRightGrabber("upBlock", "closed");
            sleep(500);
            robot.driveToPoint(0.8, 26, 0, 0, 1);
            robot.driveToPoint(0.8, 10, -210, 0, 3);
            robot.rotateAbsolute(1, 0, 2);
            robot.driveToPoint(0.8, -28, 0, 0, 2);
            robot.sideGrabber.moveRightGrabber("down", "open");
            sleep(300);
            robot.sideGrabber.moveRightGrabber("upEmpty", "open");
            robot.driveToPoint(0.8, 30, 0, 0, 1);
            robot.sideGrabber.moveRightGrabber("upEmpty", "closed");
            robot.rotateAbsolute(1, 0, 1);
            robot.driveToPoint(0.8, -10, 120, 0, 2);
            robot.rotateAbsolute(1, 0, 1);
            robot.driveToPoint(0.7, -20, 0, 0, 1);
        }
        else if (valMid == 0) { // skystone is in position 2 and 5
            robot.driveToPoint(0.8, -70, -10, 0, 2);
            robot.sideGrabber.moveRightGrabber("upEmpty", "open");
            sleep(400);
            robot.sideGrabber.moveRightGrabber("down", "open");
            sleep(400);
            robot.sideGrabber.moveRightGrabber("down", "closed");
            sleep(400);
            robot.sideGrabber.moveRightGrabber("upBlock", "closed");
            sleep(500);
            robot.driveToPoint(0.8, 20, 0, 0, 1);
            robot.driveToPoint(0.8, 5, -190, 0, 3);
            robot.rotateAbsolute(1, 0, 2);
            robot.driveToPoint(0.8, -30, 0, 0, 2);
            robot.sideGrabber.moveRightGrabber("down", "open");
            sleep(300);
            robot.sideGrabber.moveRightGrabber("upEmpty", "open");
            robot.driveToPoint(0.8, 25, 0, 0, 1);
            robot.sideGrabber.moveRightGrabber("upEmpty", "closed");
            robot.driveToPoint(0.8, 0, 110, 0, 2);
            robot.rotateAbsolute(1, 0, 1);
            robot.driveToPoint(0.7, -20, 0, 0, 1);
        }
        else { // skystone is in position 3 and 6
            robot.driveToPoint(0.8, -70, -25, 0, 2);
            robot.sideGrabber.moveRightGrabber("upEmpty", "open");
            sleep(400);
            robot.sideGrabber.moveRightGrabber("down", "open");
            sleep(400);
            robot.sideGrabber.moveRightGrabber("down", "closed");
            sleep(400);
            robot.sideGrabber.moveRightGrabber("upBlock", "closed");
            sleep(500);
            robot.driveToPoint(0.8, 16, 0, 0, 1);
            robot.driveToPoint(0.8, 8, -165, 0, 3);
            robot.rotateAbsolute(1, 0, 2);
            robot.driveToPoint(0.8, -32, 0, 0, 2);
            robot.sideGrabber.moveRightGrabber("down", "open");
            sleep(300);
            robot.sideGrabber.moveRightGrabber("upEmpty", "open");
            robot.driveToPoint(0.8, 22, 0, 0, 1);
            robot.sideGrabber.moveRightGrabber("upEmpty", "closed");
            robot.driveToPoint(0.8, -5, 100, 0, 2);
            robot.rotateAbsolute(1, 0, 1);
            robot.driveToPoint(0.7, -15, 0, 0, 1);
        }
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}