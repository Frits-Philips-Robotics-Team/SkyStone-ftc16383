package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.jetbrains.annotations.NotNull;

public class Intake {
    private CRServo leftWheel;
    private CRServo rightWheel;

    private double prevPower;

    void init(@NotNull HardwareMap hwMap) {
        leftWheel = hwMap.get(CRServo.class, "left_intake");
        rightWheel = hwMap.get(CRServo.class, "right_intake");

        rightWheel.setDirection(CRServo.Direction.REVERSE);
        prevPower = 0;
    }

    public void setPower(double power) {
        if (power != prevPower) {
            power = Range.clip(power, -1, 1);
            leftWheel.setPower(power);
            rightWheel.setPower(power);
            prevPower = power;
        }
    }
}
