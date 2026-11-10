package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.ShootingController;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

@TeleOp
public class FeedandLaunch extends RobotOpMode {


    @Override
    public void init() {
        super.init(); //calls all init() methods in RobotOpMode

        drivetrain.usePreviousStartingPose();
        vision.close(); //turns off Vision, frees up CPU
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        wrapLoop(() -> {

            // Left Bumper to toggle feed mode
            if (gamepad1.leftBumperWasPressed()) intake.toggleLaunch();

            // X Toggles flywheel on and off
            if (gamepad1.xWasPressed()) flywheel.toggle();

            // Set target RPM up by 25
            if (gamepad1.dpadUpWasReleased()) flywheel.setTarget(flywheel.getTarget() + 25);

            // Set target RPM up by 25
            if (gamepad1.dpadDownWasReleased()) flywheel.setTarget(flywheel.getTarget() - 25);

            // Auto aim unlocks if the right stick is moved
            if (Math.abs(gamepad1.right_stick_x) >= 0.1) drivetrain.unlockAutoAim();

            // A to lock auto aim on the nearest Hive
            if (gamepad1.a) drivetrain.lockAutoAim();

            // Y to Reset Odometry, intake face in at right corner
            if (gamepad1.y) drivetrain.follower.setPose(new Pose(8, 8, Math.toRadians(180)));

            // Run field centric drive - check to see if AutoHold is activated
            drivetrain.fieldCentricDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );


        });
        }
}
