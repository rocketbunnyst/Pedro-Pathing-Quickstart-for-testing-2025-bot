package org.firstinspires.ftc.teamcode.subsystems.vision;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import android.util.Size;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.vision.hivevision.BallDetectorPipeline;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Vision {
    /**
     * The variable to store our instance of the AprilTag processor/pipeline.
     */
    private final AprilTagProcessor tagProcessor;

    private final BallDetectorPipeline ballProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    private final VisionPortal visionPortal;

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

    /**
    * Arducam ezBOX-VGA GS Calibration Values from 3df Zephyr Free App
     * Global Shutter, Fixed Focus
     * MJPG Framerate 640x480@120fps
     * FOV 166°(D)x130°(H)x100°(H)
     */
    public double fx =  252.061;
    public double fy = 252.061;
    public double cx = 330.81;
    public double cy = 189.91;



    private final Context context;
    private Mode mode = Mode.STREAMINGBOTH;

    //FSM tracker variables
    private boolean camOff = false;
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
                //.setLensIntrinsics(fx, fy, cx, cy) //for Arducam. Commented out because it's in the xml calibration file

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        tagProcessor.setDecimation(3); //used 1 in Decode

        ballProcessor = new BallDetectorPipeline();

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
                .setAutoStopLiveView(true)
                .build();
    }

    // Streaming status setting commands for CPU management
    public Command stopStreaming() {
        return instant(() -> mode = Mode.STOPSTREAMING).requiring(visionPortal);
    }

    public Command hiveWatch() {
        return instant(() -> mode = Mode.HIVEWATCH).requiring(visionPortal);
    }

    public Command ballDetection() {
        return instant(() -> mode = Mode.BALLDETECTION).requiring(visionPortal);
    }

    public Command close() {
        return instant(() -> mode = Mode.CLOSED).requiring(visionPortal);
    }

    // Checks to see if the Hive is up and ready to score
    // by looking for the opposite Hive April Tag Orientation to be upside-down
    // Vision Mode must be HiveWatch to use these commands
    public Command checkIfLeftHiveUp(Alliance alliance) {
        return waitUntil(() -> isLeftHiveUp(alliance));
    }

    public Command checkIfRightHiveUp(Alliance alliance) {
        return waitUntil(() -> isRightHiveUp(alliance));
    }

    public Command periodic() { //use to control camera streaming settings to save CPU
        return infinite(() -> {
            switch (mode) {
                case STOPSTREAMING:
                    // Save CPU resources; can resume streaming when needed.
                    // check to see if stream is already off, if not, turn it off
                    if (visionPortal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_READY ||
                            visionPortal.getCameraState() != (VisionPortal.CameraState.STOPPING_STREAM)) {
                        visionPortal.stopStreaming();
                    }
                    break;
                case STREAMINGBOTH: // everything on during init so that we can use LiveView
                    if (visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY ||
                            visionPortal.getCameraState() == (VisionPortal.CameraState.STOPPING_STREAM)) {
                        visionPortal.resumeStreaming();
                    }
                    visionPortal.setProcessorEnabled(ballProcessor, true);
                    visionPortal.setProcessorEnabled(tagProcessor, true);
                    break;
                case HIVEWATCH:
                    if (visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY ||
                            visionPortal.getCameraState() == (VisionPortal.CameraState.STOPPING_STREAM)) {
                    visionPortal.resumeStreaming(); }
                    visionPortal.setProcessorEnabled(ballProcessor, false);
                    visionPortal.setProcessorEnabled(tagProcessor, true);
                    visionPortal.stopLiveView(); //don't need this during Auton
                    break;
                case BALLDETECTION:
                    if (visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY ||
                            visionPortal.getCameraState() == (VisionPortal.CameraState.STOPPING_STREAM)) {
                        visionPortal.resumeStreaming(); }
                    visionPortal.setProcessorEnabled(ballProcessor, true);
                    visionPortal.setProcessorEnabled(tagProcessor, false);
                    visionPortal.stopLiveView(); //don't need this during Auton
                    break;
                case CLOSED: // use only if not planning to use camera during OpMode. Cannot be reopened
                    visionPortal.close();
                    break;
            }


            context.telemetry.addData("Vision/mode", mode);
            context.telemetry.addData("Stream Status", visionPortal.getCameraState());
        });

    }

    private Boolean isLeftHiveUp(Alliance alliance) {
        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
            if (alliance == Alliance.RED) { //look for Red Scoring Tag to be upside down
                if (clusterDet.metadata.name.equals("RED SCORING") && clusterDet.ftcPose.roll > Math.abs(90)) {
                    context.telemetry.addData("Vision/isLeftHiveUp angle", clusterDet.ftcPose.roll);
                    return true;
                }
            } else if (alliance == Alliance.BLUE) { //look for Blue Audience Tag to be upside down
                if (clusterDet.metadata.name.equals("BLUE AUDIENCE") && clusterDet.ftcPose.roll > Math.abs(90)) {
                    context.telemetry.addData("Vision/isLeftHiveUp angle", clusterDet.ftcPose.roll);
                    return true;
                }
            }
        }
        return false;
    }

    private Boolean isRightHiveUp(Alliance alliance) {
        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
            if (alliance == Alliance.RED) { //look for Red Audience Tag to be upside down
                if (clusterDet.metadata.name.equals("RED AUDIENCE") && clusterDet.ftcPose.roll > Math.abs(90)) {
                    context.telemetry.addData("Vision/isLeftHiveUp angle", clusterDet.ftcPose.roll);
                    return true;
                }
            } else if (alliance == Alliance.BLUE) { //look for Blue Scoring Tag to be upside down
                if (clusterDet.metadata.name.equals("BLUE SCORING") && clusterDet.ftcPose.roll > Math.abs(90)) {
                    context.telemetry.addData("Vision/isLeftHiveUp angle", clusterDet.ftcPose.roll);
                    return true;
                }
            }
        }
        return false;
    }

    enum Mode {
        STOPSTREAMING,
        STREAMINGBOTH,
        HIVEWATCH,
        BALLDETECTION,
        CLOSED
    }
}
