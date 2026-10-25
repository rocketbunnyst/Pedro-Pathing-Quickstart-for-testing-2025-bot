package org.firstinspires.ftc.teamcode.autonomous;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

@Autonomous
public class PartnerHasNoAuto extends RobotOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees(); // to work in degrees!

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose shootpreload = poseFactory.of(56, 32, 90);
    private final Pose collectflowerStart = poseFactory.of(56, 32, 90);
    private final Pose collectflower = poseFactory.of(13, 48, 180);
    private final Pose shootflowerStart = poseFactory.of(13, 48, 180);
    private final Pose shootflower = poseFactory.of(57, 108, 270);
    private final Pose shootflowerControl1 = poseFactory.of(18.8295, 103.1199, 0);
    private final Pose collectflowertwoStart = poseFactory.of(57, 108, 270);
    private final Pose collectflowertwo = poseFactory.of(48, 126, 90);
    private final Pose shootflowertwoStart = poseFactory.of(48, 126, 90);
    private final Pose shootflowertwo = poseFactory.of(57, 108, 270);
    private final Pose lineupgardenStart = poseFactory.of(57, 108, 270);
    private final Pose lineupgarden = poseFactory.of(26, 9, 180);
    private final Pose lineupgardenControl1 = poseFactory.of(9, 112, 0);
    private final Pose collectgarden = poseFactory.of(10, 9, 180);
    private final Pose shootgardenStart = poseFactory.of(10, 9, 180);
    private final Pose shootgarden = poseFactory.of(56, 32, 90);
    private final Pose park = poseFactory.of(14, 94, 103.6721);
    private final Pose parkControl1 = poseFactory.of(27, 40, 0);

    public Path shootpreload() {
        return line(start, shootpreload).constant(shootpreload);
    }

    public Path collectflower() {
        return line(shootpreload, collectflower).linear(collectflowerStart, collectflower)
                .with(
                        Constants.foresightConfig.maxPathSpeed.at(0.8)
//                        Constants.foresightConfig.maxDecelerationScale.at(0.75),
//                        Constants.foresightConfig.headingDriveRatio.at(0.7)
                );
    }

    public Path shootflower() {
        return curve(collectflower, shootflowerControl1, shootflower).linear(shootflowerStart, shootflower)
 //               .with(
//                        Constants.foresightConfig.maxPathSpeed.at(0.7)
//                        Constants.foresightConfig.maxDecelerationScale.at(0.75),
//                        Constants.foresightConfig.headingDriveRatio.at(0.7)
//                )
        ;
    }

    public Path collectflowertwo() {
        return line(shootflower, collectflowertwo).linear(collectflowertwoStart, collectflowertwo);
    }

    public Path shootflowertwo() {
        return line(collectflowertwo, shootflowertwo).linear(shootflowertwoStart, shootflowertwo);
    }

    public Path lineupgarden() {
        return curve(shootflowertwo, lineupgardenControl1, lineupgarden).linear(lineupgardenStart, lineupgarden)
                .with(
                        Constants.foresightConfig.maxPathSpeed.at(0.8)
                );
    }

    public Path collectgarden() {
        return line(lineupgarden, collectgarden).constant(collectgarden);
    }

    public Path shootgarden() {
        return line(collectgarden, shootgarden).linear(shootgardenStart, shootgarden);
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
waitMs(2000) -- waits for 2 seconds (2000 Milli-Seconds)

sequential( - does each command in list order
parallel( - does each command at the same time, waiting until all finish before moving on
race( - like parallel, but as soon as 1 finishes, it cancels the others and moves on

 */

    @Override
    public void start() {
        schedule(
                sequential(
                follow(drivetrain.follower, shootpreload()),
                        waitMs(1000),
                // Add launch command here
                follow(drivetrain.follower, collectflower()),
                        waitMs(1000),
                        follow(drivetrain.follower, shootflower()),
                        waitMs(1000),
                        follow(drivetrain.follower, collectflowertwo()),
                        waitMs(1000),
                        follow(drivetrain.follower, shootflowertwo()),
                        waitMs(1000),
                        follow(drivetrain.follower, lineupgarden()),
                        waitMs(1000),
                        follow(drivetrain.follower, collectgarden()),
                        waitMs(1000),
                        follow(drivetrain.follower, shootgarden()),
                        waitMs(1000),
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
