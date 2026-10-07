package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.math.Interpolation;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.Context;

@Config
public final class Drivetrain implements AutoCloseable {
    private static Pose poseTransfer; // updates made in Periodic
    public final Follower follower;
    private final Context context;
    public double speedScalar = 0.75;
    private boolean lockAutoAim = false;

    public Drivetrain(Context context) {
        this.context = context;

        follower = Constants.create(context.hardwareMap);
    }

    public static void localize(Pose pose) {
        poseTransfer = pose;
    }

    public void lockAutoAim() {
        lockAutoAim = true;
    }

    public void unlockAutoAim() {
        lockAutoAim = false;
    }

    public void usePreviousStartingPose() {
        follower.setPose(poseTransfer);
    }

   // Field Centric Manual Drive for use in Teleop
    public void fieldCentricDrive(double forward, double lateral, double turn) {
        forward = forward * speedScalar;
        lateral = lateral * speedScalar;
        turn = turn * speedScalar;

        //Autoaim for a fixed launcher facing 0 degrees on robot
        if (lockAutoAim) {
            //write autoaim logic modifying rotate
            double targetHeading = Interpolation.getAngleToNearestHive(follower.pose());
            double currentHeading = follower.pose().heading();

            // Replace robot heading command directly with the needed rotation
            // Determine shortest rotation direction
            double headingError = Interpolation.angleWrap(targetHeading - currentHeading);

            // Turn in that direction at a fixed rate (scaled by error)
            turn = Math.copySign(Math.min(Math.abs(headingError) / Math.PI, 1.0), headingError);

            context.telemetry.addData("Drivetrain AutoAim turn power", turn);
        }

        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                lateral,
                turn,
                follower.pose().heading()
        );

        ManualDrive.driveOrHold(follower, powers); //automatically holds position when driver input stops
    }

    public void update() {
        follower.update();
    }

    public Command periodic() {
        return infinite(() -> {
            poseTransfer = follower.pose();

            context.addPose("Drivetrain/current", follower.pose());
            context.telemetry.addData("Drivetrain AutoAim locked", lockAutoAim);
        });
    }

    @Override
    public void close() {

    }
}
