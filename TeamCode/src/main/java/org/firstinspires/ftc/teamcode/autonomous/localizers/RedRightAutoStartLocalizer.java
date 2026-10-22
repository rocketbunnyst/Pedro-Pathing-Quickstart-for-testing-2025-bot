package org.firstinspires.ftc.teamcode.autonomous.localizers;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

@Utility
@Autonomous
public class RedRightAutoStartLocalizer extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drivetrain.localize(new Pose(55.59, 8, 90));
        RobotOpMode.setAlliance(Alliance.RED);
    }
}
