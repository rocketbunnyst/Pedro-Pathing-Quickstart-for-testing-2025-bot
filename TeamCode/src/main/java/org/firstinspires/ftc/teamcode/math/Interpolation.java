package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.teamcode.util.ActiveHive;

import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

public class Interpolation {

    private static final Interpolation2D flywheelVelocity = new BilinearInterpolation(
            new double[]{41.8, 71, 101.2}, // x position
            new double[]{5, 29.3},         // y position
            new double[][]{                // velocities at each point EXAMPLE VALUES, TUNE BEFORE USE!
                    {1150, 620},          // (x1, y1), (x1, y2)
                    {660, 570},          // (x2, y1), (x2, y2)
                    {575, 440},          // (x3, y1), (x3, y2)
            });

    /** Returns the interpolation value for the current location, corrected for side of the field */
    private static double interpolate(Interpolation2D interpolation, Pose pose) {
        double newPoseX = pose.x();;
        double newPoseY;
        if (pose.y() > 70.75) { // if on the left side of the field
            newPoseY = 141.5 - pose.y(); // flip the y value to be on the right side of the field
        } else { newPoseY = pose.y(); }
        return interpolation.interpolate(newPoseX, newPoseY);
    }

    /** Returns the flywheel velocity at the given pose */
    public static double getFlywheelVelocity(Pose pose) {
        return interpolate(flywheelVelocity, pose);
    }

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
