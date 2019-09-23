package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotor leftWheel;
    private DcMotor rightWheel;
    private ElapsedTime cycleTime = new ElapsedTime();

    private double prevPower;

    public void init(HardwareMap hwMap) {
        leftWheel = hwMap.get(DcMotor.class, "left_wheel");
        rightWheel = hwMap.get(DcMotor.class, "right_wheel");

        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);

        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        prevPower = 0;
    }

    public void setIntakePower(double power) {
        if (power != prevPower) {
            power = Range.clip(power, -1, 1);
            leftWheel.setPower(power);
            rightWheel.setPower(power);
            prevPower = power;
        }
    }
}
