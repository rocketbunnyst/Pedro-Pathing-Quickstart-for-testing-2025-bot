package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        c.manualBrakeMode.set(true); //for teleop usage Manual Brake Mode
    });

    public static OTOSConfig localizerConfig = new OTOSConfig(c -> {
        c.name.set("sensor_otos");
        c.linearScalar.set(0.9603544615384615);
        c.angularScalar.set(0.9909952140810432);
        c.offset.set(new Pose(1.75, 0.0));
        c.linearUnit.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.24526750760631996);
                Controller secondaryTranslationalForward = Controller.proportional(0.09061979039203133);
                Controller primaryTranslationalLateral = Controller.proportional(0.36804586473377776);
                Controller secondaryTranslationalLateral = Controller.proportional(0.13598311265250307);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01819278224356302));
                c.brake.set(Controller.proportionalFeedforward(0.015463864907028568));

                c.headingFeedback.set(Controller.proportional(3.2201692058546407));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.1595246659091917, -0.025605811837770834));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08549985659635138, 0.02986645293151578));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001486254531479092, 0.0028693001504734076));

                c.maxAchievableForwardVelocity.set(54.80618168071756);
                c.maxAchievableStrafeVelocity.set(46.17046854655611);
                c.naturalForwardDeceleration.set(37.612397323461565);
                c.naturalStrafeDeceleration.set(77.87166124263234);
            }
    );
}