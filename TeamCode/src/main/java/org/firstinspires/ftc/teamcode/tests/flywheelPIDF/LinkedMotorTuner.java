package org.firstinspires.ftc.teamcode.tests.flywheelPIDF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


// 192.168.43.1:8080/dash
//https://www.youtube.com/watch?v=NiM_WqIV0vU
@Config
@TeleOp
public class LinkedMotorTuner extends LinearOpMode {
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, 0);
    // P should be around 1.26, I and D should be 0. F should be around 12.6 +/-

    //Tune F first, start very small, like 0.0000000001, and then keep removing zeros until it starts moving,
    //and gets as close as possible to the target without overshooting. Then start tuning P, again starting very small
    //and try to smooth it out.

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    //===========MOTORS==========\\
    private DcMotorEx launcherRight;
    private DcMotorEx launcherLeft;

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Change my id
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");

        // Reverse as appropriate
        launcherRight.setDirection(DcMotorEx.Direction.FORWARD);
        launcherLeft.setDirection(DcMotorEx.Direction.REVERSE);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        MotorConfigurationType rightMotorConfigurationType = launcherRight.getMotorType().clone();
        rightMotorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        launcherRight.setMotorType(rightMotorConfigurationType);

        MotorConfigurationType leftMotorConfigurationType = launcherLeft.getMotorType().clone();
        leftMotorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        launcherLeft.setMotorType(leftMotorConfigurationType);

        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(launcherRight, MOTOR_VELO_PID);
        setPIDFCoefficients(launcherLeft, MOTOR_VELO_PID);

        TuningController tuningController = new TuningController();

        double lastKp = 0.0;
        double lastKi = 0.0;
        double lastKd = 0.0;
        double lastKf = getMotorVelocityF();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        tuningController.start();

        while (!isStopRequested() && opModeIsActive()) {
            double targetVelo = tuningController.update();
            launcherRight.setVelocity(targetVelo);
            launcherLeft.setVelocity(targetVelo);

            telemetry.addData("targetVelocity", targetVelo);

            double motorVelo = getVelocityAverage();
            telemetry.addData("velocity", motorVelo);
            telemetry.addData("error", targetVelo - motorVelo);

            telemetry.addData("upperBound", TuningController.rpmToTicksPerSecond(TuningController.TESTING_MAX_SPEED * 1.15));
            telemetry.addData("lowerBound", 0);

            if (lastKp != MOTOR_VELO_PID.p || lastKi != MOTOR_VELO_PID.i || lastKd != MOTOR_VELO_PID.d || lastKf != MOTOR_VELO_PID.f) {
                setPIDFCoefficients(launcherRight, MOTOR_VELO_PID);
                setPIDFCoefficients(launcherLeft, MOTOR_VELO_PID);

                lastKp = MOTOR_VELO_PID.p;
                lastKi = MOTOR_VELO_PID.i;
                lastKd = MOTOR_VELO_PID.d;
                lastKf = MOTOR_VELO_PID.f;
            }

            telemetry.update();
        }
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 * 60.0 / (TuningController.MOTOR_MAX_RPM * TuningController.MOTOR_TICKS_PER_REV);
    }

    public double getVelocityAverage() {
        double leftVelocity = launcherLeft.getVelocity(); // ticks/s of motor
        double rightVelocity = launcherRight.getVelocity(); // ticks/s of motor

        double averageVelocity = (leftVelocity + rightVelocity) / 2.0;

        return averageVelocity;
    }
}
