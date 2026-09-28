package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Utility
public class ExamplePPRobotCentricTeleop extends OpMode {
    private Follower follower;
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }
    @Override
    public void loop() {
        ManualDrive.driveOrHold( //automatically holds position when driver input stops
                follower,
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        follower.update();
    }
}