package org.firstinspires.ftc.teamcode.autonomous;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.RobotOpMode;

@Autonomous
public class RightSideExample extends RobotOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees(); // to work in degrees!

    private final Pose start = poseFactory.of(55.59, 8, 90);
    private final Pose toLaunchPreload = poseFactory.of(58, 14, 90);
    private final Pose toPark = poseFactory.of(12, 93, 90.4639);
    private final Pose toParkControl1 = poseFactory.of(58, 36, 0);
    private final Pose toParkControl2 = poseFactory.of(12, 36, 0);

    public Path toLaunchPreload() {
        return line(start, toLaunchPreload).constant(toLaunchPreload);
    }

    public Path toPark() {
        return curve(toLaunchPreload, toParkControl1, toParkControl2, toPark).tangent();
    }

    @Override
    public void init() {
        super.init();

        drivetrain.follower.setPose(start);
        drivetrain.follower.update();
    }

    @Override
    public void start() {
        schedule(
                sequential(
                follow(drivetrain.follower, toLaunchPreload()),
                // Add launch command here
                follow(drivetrain.follower, toPark())
        ));
    }

    @Override
    public void loop(){
        wrapLoop(() -> {
            context.telemetry.addData("Follower mode", drivetrain.follower.isBusy());

            // add your other methods needed in the loop here
        });


    }
}
