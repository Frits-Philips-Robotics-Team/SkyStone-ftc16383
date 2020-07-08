package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NotOpMode.FritsBot;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name= "DriveRightBack", group="red")
//@Disabled//comment out this line before using
public class AutonDriveRightBack extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private FritsBot robot = new FritsBot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.initAutonomous(this);

        waitForStart();
        runtime.reset();

        robot.driveToPoint(0.8, 20, 0, 0, 3);
        robot.driveToPoint(0.6, 0, -10, 0, 2);
    }
}