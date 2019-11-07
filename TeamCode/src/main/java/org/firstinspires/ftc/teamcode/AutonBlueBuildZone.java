/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.NotOpMode.FritsBot;



@Autonomous(name="BlueBuildZone", group="blue")
public class AutonBlueBuildZone extends LinearOpMode {

    private FritsBot robot = new FritsBot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.liftGrab.moveGrabber("in", "open");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.drivetrain.encoderDrive(0.6, 0.6, 0.56, -75, -75, -60, 2, this);

        robot.liftGrab.setLiftPower(0.8);
        sleep(1000) ;
        robot.liftGrab.setLiftPower(0);

        robot.liftGrab.moveGrabber("out", "open");
        sleep(500);

        robot.liftGrab.setLiftPower(-0.25);
        sleep(2200);
        robot.liftGrab.setLiftPower(0);
        sleep(500);

        robot.drivetrain.encoderDrive(0.6, 0.6, 0.0, 83, 83, 0, 2, this);

        robot.liftGrab.setLiftPower(0.8);
        sleep(900);
        robot.liftGrab.setLiftPower(0);

        robot.liftGrab.moveGrabber("in", "open");
        sleep(500);

        robot.liftGrab.setLiftPower(-0.2);
        sleep(1000);

        robot.drivetrain.encoderDrive(0.15, 0.15, 0.6, -5, -5, 150, 4, this);
        robot.liftGrab.setLiftPower(0);
        robot.sideGrabber.moveGrabber("down", "");
    }
}
