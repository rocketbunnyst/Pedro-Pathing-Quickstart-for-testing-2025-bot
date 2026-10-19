package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.Context;

public final class Flywheel {
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, 0);
    public static int velocityTolerance = 25;
    private final DcMotorEx flywheelMotorRight;
    private final DcMotorEx flywheelMotorLeft;

    private VoltageSensor batteryVoltageSensor;
    private final Context context;
    private Mode mode = Mode.OFF;
    private double target = 1500; //put at 0 once there is a shooting controller

    public Flywheel(Context context) {
        this.context = context;

        // ================== MOTORS ================== \\
        flywheelMotorRight = context.motor("launcherRight");
        flywheelMotorLeft = context.motor("launcherLeft");
        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorConfigurationType rightMotorConfigurationType = flywheelMotorRight.getMotorType().clone();
        rightMotorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        flywheelMotorRight.setMotorType(rightMotorConfigurationType);

        MotorConfigurationType leftMotorConfigurationType = flywheelMotorLeft.getMotorType().clone();
        leftMotorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        flywheelMotorLeft.setMotorType(leftMotorConfigurationType);

        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        batteryVoltageSensor = context.hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(flywheelMotorRight, MOTOR_VELO_PID);
        setPIDFCoefficients(flywheelMotorLeft, MOTOR_VELO_PID);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void off() {
        mode = Mode.OFF;
    }

    public void on() {
        mode = Mode.VELOCITY;
    }

    public void toggle() {
        mode = mode == Mode.OFF ? Mode.VELOCITY : Mode.OFF;
    }

    public double getVelocity() {
        //return flywheelMotorBottom.getVelocity();

        double leftVelocity = flywheelMotorLeft.getVelocity(); // ticks/s of motor
        double rightVelocity = flywheelMotorRight.getVelocity(); // ticks/s of motor

        double averageVelocity = (leftVelocity + rightVelocity) / 2.0;

        return averageVelocity;
    }

    public double getTarget() {
        return target;
    }

    public boolean atTarget() {
        return Math.abs(target - getVelocity()) <= velocityTolerance;
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    public Command periodic() {
        return infinite(() -> {
            switch (mode) {
                case OFF:
                    flywheelMotorRight.setVelocity(0);
                    flywheelMotorLeft.setVelocity(0);
                    return;
                case VELOCITY:
                    setPIDFCoefficients(flywheelMotorRight, MOTOR_VELO_PID);
                    setPIDFCoefficients(flywheelMotorLeft, MOTOR_VELO_PID);
                    flywheelMotorRight.setVelocity(target);
                    flywheelMotorLeft.setVelocity(target);
                    break;
            }

            context.telemetry.addData("Flywheel/velocity", getVelocity());
            context.telemetry.addData("Flywheel/target", target);

        });
    }

    enum Mode {
        OFF,
        VELOCITY
    }
}
