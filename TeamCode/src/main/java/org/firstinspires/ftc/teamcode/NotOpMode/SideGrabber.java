package org.firstinspires.ftc.teamcode.NotOpMode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class SideGrabber {
    // list of SideGrabber states
    private enum State
    {
        STATE_MOVE_TO_INIT,
        STATE_INIT_POSITION,
        STATE_MOVE_UP_OPEN,
        STATE_UP_OPEN_POSITION,
        STATE_MOVE_UP_CLOSED,
        STATE_UP_CLOSED_POSITION,
        STATE_MOVE_DOWN_OPEN,
        STATE_DOWN_OPEN_POSITION,
        STATE_MOVE_DOWN_CLOSED,
        STATE_DOWN_CLOSED_POSTION,
    }

    // TODO: find out which values the servos need for these positions
    private final double upValue = 1;
    private final double downValue = 0;
    private final double openValue = 1;
    private final double closedValue = 0;

    private Servo armAngle;
    private Servo gripper;
    private ElapsedTime cycleTime = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        armAngle = hwMap.get(Servo.class, "side_arm");
        gripper = hwMap.get(Servo.class, "side_gripper");

        cycleTime.reset();
    }
}
