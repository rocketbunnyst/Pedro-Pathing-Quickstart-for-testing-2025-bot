package org.firstinspires.ftc.teamcode.tests;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Context;

import java.util.List;
@Utility
public class VisionTestOpMode extends OpMode {
    private static Alliance currentAlliance = Alliance.RED;

    protected Vision vision;
    protected Context context;

    protected static Alliance alliance() {
        return currentAlliance;
    }

    public static void setAlliance(Alliance alliance) {
        currentAlliance = alliance;
    }

    private List<LynxModule> hubs;
    private final ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void init() {
        Scheduler.reset();
        context = new Context(this);

       vision = new Vision(context);

        schedule(

                vision.periodic()
        );

        hubs = hardwareMap.getAll(LynxModule.class);

        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
    }

    @Override
    public void init_loop() {
        context.telemetry.addData("Robot/loop time", loopTimer.milliseconds());
        loopTimer.reset();
        context.telemetry.addData("Robot/alliance", alliance());

        telemetry.addData("Press X", "for BLUE Alliance");
        telemetry.addData("Press B", "for RED Alliance");
        if (gamepad1.x) { setAlliance(Alliance.BLUE); }
        if (gamepad1.b) { setAlliance(Alliance.RED); }

        Scheduler.execute();
        context.telemetry.update();
    }

    @Override
    public void loop() {
        wrapLoop(() -> {


        });
    }

    protected final void wrapLoop(Runnable runnable) {
        context.telemetry.addData("Robot/loop time", loopTimer.milliseconds());
        loopTimer.reset();
        hubs.forEach(LynxModule::clearBulkCache);
        context.telemetry.addData("Robot/alliance", alliance());

        runnable.run();

        Scheduler.execute();
        context.telemetry.update();
    }



}
