package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.ActiveHive;

public class Interpolation {

    /** Returns the absolute heading (radians) to the nearest Hive */
    public static double getAngleToNearestHive(Pose currentPosition) { //Angle to closet Hive in Radians
        Pose target;
        if (currentPosition.y() < 70.75) {
            target = ActiveHive.RIGHT.goal;
        } else {
            target = ActiveHive.LEFT.goal;
        }

        double angle = Math.atan2(target.y() - currentPosition.y(), target.x() - currentPosition.x()) ;
        return angle;
    }

    /** Helper to wrap angles to [-PI, PI] */
    public static double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle; }
}
