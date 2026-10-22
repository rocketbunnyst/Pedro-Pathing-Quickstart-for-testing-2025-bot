package org.firstinspires.ftc.teamcode.autonomous;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
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

    /*  AVAILABLE COMMANDS

    instant(flywheel::on) -- turns on the flywheel at the beginning of the auto
    waitUntil(flywheel::atTarget) -- waits until the flywheel is at the target velocity
    intake.launch3() -- feeds balls into the launcher for 2 seconds
    instant(intake::intake) -- starts the intake sequence - it will run until it gets 3 balls, make sure to turn it off manually as a safeguard!
    instant(intake::off) -- turns off the intake
    waitMs(2000) -- waits for 2 seconds (2000 Milli-Seconds)

    sequential( - does each command in list order
    parallel( - does each command at the same time, waiting until all finish before moving on
    race( - like parallel, but as soon as 1 finishes, it cancels the others and moves on

     */

    @Override
    public void start() {
        schedule(
                sequential(
                        instant(flywheel::on),
                        follow(drivetrain.follower, toLaunchPreload()),
                        waitUntil(flywheel::atTarget),
                        intake.launch3(),
                        follow(drivetrain.follower, toPark()),
                        instant(flywheel::off)
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
