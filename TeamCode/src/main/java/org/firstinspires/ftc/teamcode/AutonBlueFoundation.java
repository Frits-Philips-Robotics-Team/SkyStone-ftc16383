package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NotOpMode.FritsBot;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name= "BlueFoundation", group="blue")
//@Disabled//comment out this line before using
public class AutonBlueFoundation extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private FritsBot robot = new FritsBot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.initAutonomous(this);

        waitForStart();
        runtime.reset();

        robot.driveToPoint(1, 20, -75, 0, 3);
        robot.foundationServo.moveUp(false);
        sleep(1000);
        robot.driveToPoint(1, 0, 85, 0, 3);
        robot.rotateAbsolute(1, 0, 2);
        robot.driveToPoint(0.7, 0, 20, 0, 2);
        robot.foundationServo.moveUp(true);
        sleep(500);
        robot.driveToPoint(0.8, 0, 5, 0, 1);
        robot.driveToPoint(0.8, -120, 0, 0, 3);
        robot.driveToPoint(0.7, 0, 20, 0, 1);
    }
}