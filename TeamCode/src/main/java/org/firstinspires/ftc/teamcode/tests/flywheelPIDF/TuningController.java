package org.firstinspires.ftc.teamcode.tests.flywheelPIDF;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

// Changes set velocity to several different values to help tune a PIDF controller for a Flywheel.
// Use with LinkedMotorTuner & SingleMotorTuner
// See https://github.com/NoahBres/VelocityPIDTuningTutorial
// https://www.youtube.com/watch?v=NiM_WqIV0vU

@Config
public class TuningController {
    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_MAX_RPM = 1500; //was 5400 when wheel came apart
    public static double MOTOR_GEAR_RATIO = 1;

    public static double TESTING_MAX_SPEED = 0.9 * MOTOR_MAX_RPM;
    public static double TESTING_MIN_SPEED = 0.3 * MOTOR_MAX_RPM;

    public static double ZSTATE1_RAMPING_UP_DURATION = 3.5;
    public static double ZSTATE2_COASTING_1_DURATION = 4;
    public static double ZSTATE3_RAMPING_DOWN_DURATION = 2;
    public static double ZSTATE4_COASTING_2_DURATION = 2;
    public static double ZSTATE5_RANDOM_1_DURATION = 2;
    public static double ZSTATE6_RANDOM_2_DURATION = 2;
    public static double ZSTATE7_RANDOM_3_DURATION = 2;
    public static double ZSTATE8_REST_DURATION = 1;

    enum State {
        RAMPING_UP,
        COASTING_1,
        RAMPING_DOWN,
        COASTING_2,
        RANDOM_1,
        RANDOM_2,
        RANDOM_3,
        REST
    }

    private State state = State.RAMPING_UP;
    private ElapsedTime externalTimer = new ElapsedTime();
    private boolean initialized = false;

    private double currentTargetVelo = 0.0;

    public TuningController() {
        // Constructor remains empty
    }

    public void start() {
        externalTimer.reset();
        initialized = false;
        state = State.RAMPING_UP;
    }

    public double update() {
        switch (state) {
            case RAMPING_UP:
                if (!initialized) {
                    externalTimer.reset();
                    initialized = true;
                }
                double progress1 = externalTimer.seconds() / ZSTATE1_RAMPING_UP_DURATION;
                double target = progress1 * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED;

                currentTargetVelo = rpmToTicksPerSecond(target);

                if (externalTimer.seconds() >= ZSTATE1_RAMPING_UP_DURATION) {
                    initialized = false;
                    state = State.COASTING_1;
                }
                break;

            case COASTING_1:
                if (!initialized) {
                    externalTimer.reset();
                    initialized = true;
                }
                currentTargetVelo = rpmToTicksPerSecond(TESTING_MAX_SPEED);

                if (externalTimer.seconds() >= ZSTATE2_COASTING_1_DURATION) {
                    initialized = false;
                    state = State.RAMPING_DOWN;
                }
                break;

            case RAMPING_DOWN:
                if (!initialized) {
                    externalTimer.reset();
                    initialized = true;
                }
                double progress3 = externalTimer.seconds() / ZSTATE3_RAMPING_DOWN_DURATION;
                double target3 = TESTING_MAX_SPEED - progress3 * (TESTING_MAX_SPEED - TESTING_MIN_SPEED);

                currentTargetVelo = rpmToTicksPerSecond(target3);

                if (externalTimer.seconds() >= ZSTATE3_RAMPING_DOWN_DURATION) {
                    initialized = false;
                    state = State.COASTING_2;
                }
                break;

            case COASTING_2:
                if (!initialized) {
                    externalTimer.reset();
                    initialized = true;
                }
                currentTargetVelo = rpmToTicksPerSecond(TESTING_MIN_SPEED);

                if (externalTimer.seconds() >= ZSTATE4_COASTING_2_DURATION) {
                    initialized = false;
                    state = State.RANDOM_1;
                }
                break;

            case RANDOM_1:
                if (!initialized) {
                    externalTimer.reset();
                    initialized = true;
                    currentTargetVelo = rpmToTicksPerSecond(Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED);
                }

                if (externalTimer.seconds() >= ZSTATE5_RANDOM_1_DURATION) {
                    initialized = false;
                    state = State.RANDOM_2;
                }
                break;

            case RANDOM_2:
                if (!initialized) {
                    externalTimer.reset();
                    initialized = true;
                    currentTargetVelo = rpmToTicksPerSecond(Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED);
                }

                if (externalTimer.seconds() >= ZSTATE6_RANDOM_2_DURATION) {
                    initialized = false;
                    state = State.RANDOM_3;
                }
                break;

            case RANDOM_3:
                if (!initialized) {
                    externalTimer.reset();
                    initialized = true;
                    currentTargetVelo = rpmToTicksPerSecond(Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED);
                }

                if (externalTimer.seconds() >= ZSTATE7_RANDOM_3_DURATION) {
                    initialized = false;
                    state = State.REST;
                }
                break;

            case REST:
                if (!initialized) {
                    externalTimer.reset();
                    initialized = true;
                }
                currentTargetVelo = 0;

                if (externalTimer.seconds() >= ZSTATE8_REST_DURATION) {
                    initialized = false;
                    state = State.RAMPING_UP;
                }
                break;
        }

        return currentTargetVelo;
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }
}