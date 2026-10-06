package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

public enum ActiveHive {
    RIGHT(new Pose(58, 55)),
    LEFT(new Pose(58, 85.75)),
    TRANSITIONING(null),
    UNKNOWN(null);

    public static ActiveHive current = ActiveHive.RIGHT;
    public final Pose goal;

    ActiveHive(Pose goal) {
        this.goal = goal;
    }
}
