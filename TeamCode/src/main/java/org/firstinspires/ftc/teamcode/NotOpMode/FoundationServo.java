package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FoundationServo {

    private Servo left_servo;
    private Servo right_servo;
    public void init(HardwareMap hwMap) {
        left_servo = hwMap.get(Servo.class, "left_foundation");
        right_servo = hwMap.get(Servo.class, "right_foundation");

        moveUp(true);
    }

    public void moveUp(boolean servoUp) {
        final double leftUpValue = 0.8;
        final double rightUpValue = 0;
        final double leftDownValue = 0.03;
        final double rightDownValue = 0.78;

        if(servoUp) {
            left_servo.setPosition(leftUpValue);
            right_servo.setPosition(rightUpValue);
        }
        else {
            left_servo.setPosition(leftDownValue);
            right_servo.setPosition(rightDownValue);
        }
    }
}
