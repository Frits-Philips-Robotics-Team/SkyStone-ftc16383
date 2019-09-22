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

package org.firstinspires.ftc.teamcode.NotOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific hardware for a single robot.
 * In this case that robot is an H-drive robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Middle drive motor:       "mid_drive"
 */
public class HardwareH_Drive
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  midDrive    = null;
    double gearRatioLAndR = 1.3; // 20/26
    double gearRatioMid = 1.33333333333333; // 60/45
    double driveRatioLAndR = (gearRatioLAndR * 1120 / 9); // encoder counts per cm of robot movement
    double driveRatioMid = (gearRatioMid * 1120 / 9); // encoder counts per cm of robot movement

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareH_Drive(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        midDrive   = hwMap.get(DcMotor.class, "mid_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        midDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        midDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        midDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        midDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void move(double heading, int speed, int distance) {
        // Input any number 0-359 as a heading in degrees going clockwise with 0 being straight
        // forwards. Then give a speed and a distance. The function will calculate what speeds to
        // give the wheels. If distance is 0, the robot will just run at the right speed indefinitely.
        double forwardsSpeedRatio;
        double RightSpeedRatio;
        int lAndRTargetPos;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        midDrive.setPower(0);

        if (heading <= 45 ||  heading >= 315) {     //using tan always maximizes speed. Using sin always keeps speed consistent.
            //RightSpeedRatio = Math.tan(Math.toRadians(heading));
            RightSpeedRatio = Math.sin(Math.toRadians(heading));
            forwardsSpeedRatio = 1;

	        lAndRTargetPos = ((int) (driveRatioLAndR * (distance * Math.cos(Math.toRadians(heading)))));
	        leftDrive.setTargetPosition(lAndRTargetPos);
	        rightDrive.setTargetPosition(lAndRTargetPos);
	        midDrive.setTargetPosition((int) (driveRatioMid * (distance * Math.cos(Math.toRadians(heading)))));
        }
        else if (heading >= 135 && heading <= 225) {
            //RightSpeedRatio = (-1 * Math.tan(Math.toRadians(heading)));
            RightSpeedRatio = (-1 * Math.sin(Math.toRadians(heading)));
            forwardsSpeedRatio = -1;
        }
        else if (heading <= 90) {
            RightSpeedRatio = 1;
            //forwardsSpeedRatio = Math.tan(Math.toRadians(90 - heading));
            forwardsSpeedRatio = Math.sin(Math.toRadians(90 - heading));
        }
        else if (heading < 135) {
            RightSpeedRatio = 1;
            //forwardsSpeedRatio = (-1 * Math.tan(Math.toRadians(heading - 90)));
            forwardsSpeedRatio = (-1 * Math.sin(Math.toRadians(heading - 90)));
        }
        else if (heading <= 270) {
            RightSpeedRatio = -1;
            //forwardsSpeedRatio = (-1 * Math.tan(Math.toRadians(heading - 225)));
            forwardsSpeedRatio = Math.sin(Math.toRadians(heading - 270));
        }
        else {
            RightSpeedRatio = -1;
            //forwardsSpeedRatio = Math.tan(Math.toRadians(heading - 270));
            forwardsSpeedRatio = Math.sin(Math.toRadians(heading - 270));
        }

        if (distance == 0) {
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            midDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else {
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            midDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            midDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        leftDrive.setPower(Range.clip(forwardsSpeedRatio * speed, -1, 1));
        rightDrive.setPower(Range.clip(forwardsSpeedRatio * speed, -1, 1));
        midDriveGood(Range.clip(RightSpeedRatio * speed, -1, 1));
    }

    public void midDriveGood(double power){
        midDrive.setPower(0.96153846153846 * power);
    }
 }

