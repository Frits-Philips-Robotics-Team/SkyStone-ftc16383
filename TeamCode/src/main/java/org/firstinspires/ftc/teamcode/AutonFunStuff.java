package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NotOpMode.FritsBot;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name= "spintest", group="red")
//@Disabled//comment out this line before using
public class AutonFunStuff extends LinearOpMode {
    private FritsBot robot = new FritsBot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.initAutonomous(this);

        waitForStart();
        robot.rotateAbsolute(1, 180, 30, telemetry);
    }

}