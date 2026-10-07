package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

@TeleOp
public class Teleop14133v1 extends RobotOpMode {

    @Override
    public void init() {
        super.init(); //calls all init() methods in RobotOpMode

        drivetrain.usePreviousStartingPose();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        wrapLoop(() -> {

            // Auto aim unlocks if the right stick is moved
            if (Math.abs(gamepad1.right_stick_x) >= 0.1) drivetrain.unlockAutoAim();

            // A to lock auto aim on the nearest Hive
            if (gamepad1.a) drivetrain.lockAutoAim();

            // Reset Odometry, intake face in at right corner
            if (gamepad1.y) drivetrain.follower.setPose(new Pose(8, 8, Math.toRadians(180)));

            // Run field centric drive with AutoHold
            drivetrain.fieldCentricDrive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
        });
        }
}
