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
public class PartnerShootsOnlyShootsPreload extends RobotOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees(); // to work in degrees!

    private final Pose start = poseFactory.of(59.2991, 133.9595, 90);
    private final Pose shootPreload = poseFactory.of(59, 108, 90);
    private final Pose collectflower = poseFactory.of(39.6, 131.5, -90);
    private final Pose secondshoot = poseFactory.of(59, 108, 90);
    private final Pose collectsecondflower = poseFactory.of(10.8, 50.4, 0);
    private final Pose collectsecondflowerControl1 = poseFactory.of(23.7066, 106.4451, 0);
    private final Pose thirdshoot = poseFactory.of(56, 32, 270);
    private final Pose lineupgarden = poseFactory.of(27.3, 8.9, 0);
    private final Pose getgarden = poseFactory.of(10.1, 9, 0);
    private final Pose fourthshoot = poseFactory.of(56, 32, 270);
    private final Pose park = poseFactory.of(12.4, 96.3, -83.4749);
    private final Pose parkControl1 = poseFactory.of(17.5694, 49.5043, 0);

    public Path shootpreload() {
        return line(start, shootPreload).constant(shootPreload);
    }

    public Path collectflower() {
        return line(shootPreload, collectflower).constant(collectflower);
    }

    public Path secondshoot() {
        return line(collectflower, secondshoot).constant(secondshoot);
    }

    public Path collectsecondflower() {
        return curve(secondshoot, collectsecondflowerControl1, collectsecondflower).tangent();
    }

    public Path thirdshoot() {
        return line(collectsecondflower, thirdshoot).constant(thirdshoot);
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
                follow(drivetrain.follower, collectflower())
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
