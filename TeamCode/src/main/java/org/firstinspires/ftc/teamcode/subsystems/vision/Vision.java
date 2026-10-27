package org.firstinspires.ftc.teamcode.subsystems.vision;

import static com.pedropathing.ivy.commands.Commands.infinite;

import android.util.Size;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.vision.hivevision.BallDetectorPipeline;
import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Vision {
    /**
     * The variable to store our instance of the AprilTag processor/pipeline.
     */
    private AprilTagProcessor tagProcessor;

    private BallDetectorPipeline ballProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public WebcamName webCam;

    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     * <p>
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     * <p>
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     * <p>
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     * <p>
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 8.5, 10.5, 0);
    YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private final Context context;
    private Mode mode = Mode.OFF;
    public Vision(Context context) {
        this.context = context;

        webCam = context.hardwareMap.get(WebcamName.class, "Webcam 1");

        tagProcessor = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
               .setDrawAxes(true) // Changed in V12.0
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setDrawTagID(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                //.setDrawCubeProjection(false)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        tagProcessor.setDecimation(3); //used 1 in Decode

        visionPortal = new VisionPortal.Builder()
                // Set and enable the processor.
                .addProcessor(tagProcessor)
                .addProcessor(ballProcessor)
                .setCamera(webCam)
                // Choose a camera resolution. Not all cameras support all resolutions.
                .setCameraResolution(new Size(640, 480))
                // Set the stream format; MJPEG uses less bandwidth than default YUY2.
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
                .enableLiveView(true)
                .build();
    }

    public Command periodic() {
        return infinite(() -> {
            switch (mode) {
                case OFF:
                    // Save CPU resources; can resume streaming when needed.
                    // visionPortal.stopStreaming(); write into initialization code, use ivy command builder
                    return;
                case HIVEWATCH:
                    // visionPortal.resumeStreaming(); write into initialization code for this mode
                    // write out what to do to check for a hive
                    break;
                case BALLDETECTION:
                    // write out what to do to check for balls
                    break;
            }

            context.telemetry.addData("Vision/mode", mode);
        });

    }

    enum Mode {
        OFF,
        HIVEWATCH,
        BALLDETECTION
    }
}
