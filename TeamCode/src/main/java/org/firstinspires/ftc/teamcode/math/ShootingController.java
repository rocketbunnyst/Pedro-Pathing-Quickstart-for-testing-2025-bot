package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.util.Context;

@Config("Shooting Controller")
public final class ShootingController {
    private final Context context;
    private final Flywheel flywheel;

    public ShootingController(Context context, Flywheel flywheel) {
        this.context = context;
        this.flywheel = flywheel;
    }

    public void prepareForLocation(Pose robotPose, Vector robotVelocity, double angularVelocity) {
        double flywheelVelocity = Interpolation.getFlywheelVelocity(robotPose);

        flywheel.setTarget(flywheelVelocity);
    }
}
