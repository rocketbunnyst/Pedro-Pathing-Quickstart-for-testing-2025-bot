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
public class PartnerShootsOnlyShootsPreload extends RobotOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees(); // to work in degrees!


    private final Pose start = poseFactory.of(60, 132, 270);
    private final Pose shootPreload = poseFactory.of(59, 108, 270);
    private final Pose collectflowerStart = poseFactory.of(59, 108, 270);
    private final Pose collectflower = poseFactory.of(40, 127, 90);
    private final Pose secondshootStart = poseFactory.of(40, 127, 90);
    private final Pose secondshoot = poseFactory.of(59, 108, 270);
    private final Pose point4Start = poseFactory.of(59, 108, 270);
    private final Pose point4 = poseFactory.of(15.6416, 108.139, 180);
    private final Pose point5 = poseFactory.of(15.6615, 47.3869, 180);
    private final Pose thirdshootStart = poseFactory.of(15.6615, 47.3869, 180);
    private final Pose thirdshoot = poseFactory.of(56, 37, 90);
    private final Pose lineupgardenStart = poseFactory.of(56, 37, 90);
    private final Pose lineupgarden = poseFactory.of(43, 23, 180);
    private final Pose getgarden = poseFactory.of(17, 22, 180);
    private final Pose fourthshootStart = poseFactory.of(17, 22, 180);
    private final Pose fourthshoot = poseFactory.of(56, 37, 90);
    private final Pose park = poseFactory.of(12.4, 96.3, 96.5285);
    private final Pose parkControl1 = poseFactory.of(17.5694, 49.5043, 0);

    public Path shootpreload() {
        return line(start, shootPreload).constant(shootPreload);
    }

    public Path collectflower() {
        return line(shootPreload, collectflower).linear(collectflowerStart, collectflower);
    }

    public Path secondshoot() {
        return line(collectflower, secondshoot).linear(secondshootStart, secondshoot);}

    public Path path4() {
        return line(secondshoot, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return line(point4, point5).constant(point5);
    }

    public Path thirdshoot() {
        return line(point5, thirdshoot).linear(thirdshootStart,thirdshoot);
    }

    public Path lineupgarden() {return line(thirdshoot, lineupgarden).linear(lineupgardenStart, lineupgarden);
    }

    public Path getgarden() {return line(lineupgarden, getgarden).constant(getgarden);
    }

    public Path fourthshoot() {return line(getgarden, fourthshoot).linear(fourthshootStart, fourthshoot);
    }

    public Path park() {return curve(fourthshoot, parkControl1, park).reverseTangent();
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
                // Add launch command here
                follow(drivetrain.follower, collectflower()),
                        waitMs(2000),
                        follow(drivetrain.follower, secondshoot()),
                        waitMs(2000),
                        follow(drivetrain.follower, path4()),
                        waitMs(2000),
                        follow(drivetrain.follower, path5()),
                        waitMs(2000),
                        follow(drivetrain.follower, thirdshoot()),
                        waitMs(2000),
                        follow(drivetrain.follower, lineupgarden()),
                        waitMs(2000),
                        follow(drivetrain.follower, getgarden()),
                        waitMs(2000),
                        follow(drivetrain.follower, fourthshoot()),
                        waitMs(2000),
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
