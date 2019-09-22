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

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.NotOpMode.HardwareH_Drive;

@TeleOp(name="TankDrive", group="Iterative Opmode")
public class TankDrive extends OpMode {
    // Declare OpMode members.
    private HardwareH_Drive robot   = new HardwareH_Drive();
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime cycleTime = new ElapsedTime();

    private static final double ACCELERATION = 0.08;   // amount to ramp motor each cycle
    private static final int    CYCLE_MS  =   40;   // period of each cycle
    private double              speedMultiplier;

    private double leftPowerCurrent;
    private double rightPowerCurrent;
//    private boolean aWasPressed;
//    private boolean bWasPressed;
//    private boolean xWasPressed;
//    private boolean yWasPressed;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        robot.init(hardwareMap);
        leftPowerCurrent = 0;
        rightPowerCurrent = 0;
//        aWasPressed = false;
//        bWasPressed = false;
//        xWasPressed = false;
//        yWasPressed = false;
        speedMultiplier = 0.5;

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        cycleTime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // control wheelpower with acceleration
        if (cycleTime.milliseconds() > CYCLE_MS) {
            double leftPowerTarget = Range.clip(-gamepad1.left_stick_y + gamepad1.right_stick_x, -1, 1);
            double rightPowerTarget = Range.clip(-gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1);

            if (leftPowerTarget > leftPowerCurrent) {
                leftPowerCurrent = Range.clip(leftPowerCurrent + ACCELERATION, -1, leftPowerTarget);
                robot.leftDrive.setPower(speedMultiplier * leftPowerCurrent);
            }
            else if (leftPowerTarget < leftPowerCurrent) {
                leftPowerCurrent = Range.clip(leftPowerCurrent - ACCELERATION, leftPowerTarget, 1);
                robot.leftDrive.setPower(speedMultiplier * leftPowerCurrent);
            }
            else {
                robot.leftDrive.setPower(speedMultiplier * leftPowerCurrent);
            }

            if (rightPowerTarget > rightPowerCurrent) {
                rightPowerCurrent = Range.clip(rightPowerCurrent + ACCELERATION, -1, rightPowerTarget);
                robot.rightDrive.setPower(speedMultiplier * rightPowerCurrent);
            }
            else if (rightPowerTarget < rightPowerCurrent) {
                rightPowerCurrent = Range.clip(rightPowerCurrent - ACCELERATION, rightPowerTarget, 1);
                robot.rightDrive.setPower(speedMultiplier * rightPowerCurrent);
            }
            else {
                robot.rightDrive.setPower(speedMultiplier * rightPowerCurrent);
            }

            cycleTime.reset();
        }

        if (gamepad1.a) {
            speedMultiplier = 0.5;
        }
        if (gamepad1.y) {
            speedMultiplier = 1;
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Speed", speedMultiplier);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}