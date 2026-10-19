package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Context;

public final class Intake {
    public static boolean logCurrent = false;
    public static double intakePower = 0.5;
    public static double offPower = 0;
    public static double outtakePower = -0.5;
    public static double shortReverseTimeMs = 150;
    public static double possessionDistanceMid = 4.5;

    // ================== MOTORS ================== \\
    private final DcMotorEx intakeFront;
    private final DcMotorEx intakeRear;

    // ================== SERVOS ================== \\
    private final CRServo leftTransfer;
    private final CRServo rightTransfer;

    // ================== SENSORS ================== \\
    private final RevColorSensorV3 midColorSens;
    private final DigitalChannel frontBeamBreak;
    private final DigitalChannel rearBeamBreak;

    private final Context context;
    private Mode mode = Mode.OFF;

    // Possession and Color Variables for each auto intake round. Starts true for preload. Resets and reads during intake!
    boolean possessionFront = true;
    boolean possessionMid = true;
    boolean possessionRear = true;

    //FSM tracker variables
    private boolean initialized = false;

    public Intake(Context context) {
        this.context = context;

        // ================== MOTORS ================== \\
        intakeFront = context.motor("intakeFront");
        intakeRear = context.motor("intakeRear");

        intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRear.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ================== SERVOS ================== \\
        leftTransfer = context.hardwareMap.get(CRServo.class, "leftTransfer");
        rightTransfer = context.hardwareMap.get(CRServo.class, "rightTransfer");

        leftTransfer.setDirection(CRServo.Direction.REVERSE);
        rightTransfer.setDirection(CRServo.Direction.FORWARD);

        // ================== SENSORS ================== \\
        midColorSens = context.hardwareMap.get(RevColorSensorV3.class, "midColor");
        midColorSens.setGain(10);

        frontBeamBreak = context.hardwareMap.get(DigitalChannel.class, "frontBB");
        rearBeamBreak = context.hardwareMap.get(DigitalChannel.class, "rearBB");

        frontBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        rearBeamBreak.setMode(DigitalChannel.Mode.INPUT);
    }

    public Command launch() {
        return instant(() -> mode = Mode.LAUNCH).requiring(intakeFront, intakeRear, leftTransfer, rightTransfer);
    }

    public Command intake() {
        return instant(() -> mode = Mode.INTAKE).requiring(intakeFront, intakeRear, leftTransfer, rightTransfer);
    }

    public Command off() {
        return instant(() -> mode = Mode.OFF).requiring(intakeFront, intakeRear, leftTransfer, rightTransfer);
    }

    public Command reverse() {
        return instant(() -> mode = Mode.REVERSE).requiring(intakeFront, intakeRear, leftTransfer, rightTransfer);
    }

    public Command shortReverse() {
        return reverse().then(waitMs(shortReverseTimeMs)).then(intake()).setConflictBehavior(ConflictBehavior.OVERRIDE);
    }

    public Command launch3() {
        return launch().then(waitMs(2000)).then(off()).setConflictBehavior(ConflictBehavior.OVERRIDE);
    }

    public Command toggleIntake() { // if mode is not Intake, run intake. Else, run off.
        return conditional(() -> mode != Mode.INTAKE, intake(), off());
    }

    public Command periodic() {
        return infinite(() -> {
            switch (mode) {
                case LAUNCH:
                    intakeFront.setPower(intakePower);
                    intakeRear.setPower(intakePower);
                    leftTransfer.setPower(-1);
                    rightTransfer.setPower(-1);
                    initialized = false;
                    break;
                case OFF:
                    intakeFront.setPower(offPower);
                    intakeRear.setPower(offPower);
                    leftTransfer.setPower(0);
                    rightTransfer.setPower(0);
                    initialized = false;
                    break;
                case REVERSE:
                    intakeFront.setPower(outtakePower);
                    intakeRear.setPower(intakePower);
                    leftTransfer.setPower(1);
                    rightTransfer.setPower(1);
                    initialized = false;
                    break;
                case INTAKE:
                    if (!initialized) { // powers on intake, if it is not on
                        clearIntakePossessions();
                        intakeFront.setPower(intakePower);
                        intakeRear.setPower(outtakePower);
                        leftTransfer.setPower(1);
                        rightTransfer.setPower(1);
                        initialized = true;
                    }
                    if (rearPossession() && !possessionRear) {
                        possessionRear = true;
                        intakeRear.setPower(offPower);
                    }
                    if (possessionRear && midPossession() && !possessionMid) {
                        possessionMid = true;
                    }
                    if (possessionRear && possessionMid && frontPossession()) {
                        intakeFront.setPower(offPower);
                        leftTransfer.setPower(0);
                        rightTransfer.setPower(0);
                        possessionFront = true;
                        initialized = false;
                        mode = Mode.OFF;
                    }
                    break;
            }

            if (logCurrent) context.telemetry.addData("Intake/current", intakeFront.getCurrent(CurrentUnit.AMPS));
            context.telemetry.addData("Intake/velocity", intakeFront.getVelocity());
            context.telemetry.addData("Intake/power", intakeFront.getPower());
        });
    }

    public double getCurrent() {
        return intakeFront.getCurrent(CurrentUnit.AMPS);
    }

    public void clearIntakePossessions() { // clear Intake load possession values
        possessionFront = false;
        possessionMid = false;
        possessionRear = false;
    }

    public boolean frontPossession() { //returns true if an artifact breaks beam
        return !frontBeamBreak.getState();
    }

    public boolean midPossession() { // returns true if there is an artifact in distance
        return midColorSens.getDistance(DistanceUnit.CM) < possessionDistanceMid;
    }

    public boolean rearPossession() { //returns true if an artifact breaks beam
        return !rearBeamBreak.getState();
    }

    enum Mode {
        LAUNCH,
        INTAKE,
        OFF,
        REVERSE
    }
}
