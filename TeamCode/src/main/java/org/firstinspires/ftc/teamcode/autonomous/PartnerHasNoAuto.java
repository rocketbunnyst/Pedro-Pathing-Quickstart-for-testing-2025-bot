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
public class PartnerHasNoAuto extends RobotOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees(); // to work in degrees!

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose shootpreload = poseFactory.of(56, 32, 90);
    private final Pose collectflower = poseFactory.of(9.9277, 48.552, 180);
    private final Pose shootflower = poseFactory.of(59, 108, 270);
    private final Pose shootflowerControl1 = poseFactory.of(18.8295, 103.1199, 0);
    private final Pose collectflowertwo = poseFactory.of(50, 134, 90);
    private final Pose shootflowertwo = poseFactory.of(59, 108, 270);
    private final Pose lineupgarden = poseFactory.of(26, 9, 180);
    private final Pose lineupgardenControl1 = poseFactory.of(9, 112, 0);
    private final Pose collectgarden = poseFactory.of(9, 9, 180);
    private final Pose shootgarden = poseFactory.of(56, 32, 90);
    private final Pose park = poseFactory.of(14, 94, -76.3279);
    private final Pose parkControl1 = poseFactory.of(27, 40, 0);

    public Path shootpreload() {
        return line(start, shootpreload).constant(shootpreload);
    }

    public Path collectflower() {
        return line(shootpreload, collectflower).constant(collectflower);
    }

    public Path shootflower() {
        return curve(collectflower, shootflowerControl1, shootflower).tangent();
    }

    public Path collectflowertwo() {
        return line(shootflower, collectflowertwo).constant(collectflowertwo);
    }

    public Path shootflowertwo() {
        return line(collectflowertwo, shootflowertwo).constant(shootflowertwo);
    }

    public Path lineupgarden() {
        return curve(shootflowertwo, lineupgardenControl1, lineupgarden).tangent();
    }

    public Path collectgarden() {
        return line(lineupgarden, collectgarden).constant(collectgarden);
    }

    public Path shootgarden() {
        return line(collectgarden, shootgarden).constant(shootgarden);
    }

    public Path park() {
        return curve(shootgarden, parkControl1, park).tangent();
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
instant(intake::intake) -- starts the intake sequence - it will run until it gets 3 balls or is turned off manually
instant(intake::off) -- turns off the intake

sequential( - does each command in list order
parallel( - does each command at the same time, waiting until all finish before moving on
race( - like parallel, but as soon as 1 finishes, it cancels the others and moves on

 */

    @Override
    public void start() {
        schedule(
                sequential(
                follow(drivetrain.follower, shootpreload()),
                // Add launch command here
                follow(drivetrain.follower, collectflower()),
                        follow(drivetrain.follower, shootflower()),
                        follow(drivetrain.follower, collectflowertwo()),
                        follow(drivetrain.follower, shootflowertwo()),
                        follow(drivetrain.follower, lineupgarden()),
                        follow(drivetrain.follower, collectgarden()),
                        follow(drivetrain.follower, shootgarden()),
                        follow(drivetrain.follower, park())

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
