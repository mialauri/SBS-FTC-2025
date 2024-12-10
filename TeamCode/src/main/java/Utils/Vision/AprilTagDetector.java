/*******************************************************************************
 * AprilTagDetector.java, 2023/2/10 下午11:28.
 * Copyright (c) 2023 Xuefei (William) Tao. All Rights Reserved.
 ******************************************************************************/

package org.firstinspires.ftc.teamcode.Utils.Vision;

//import static org.firstinspires.ftc.teamcode.Officials.KodiakDriveConstants.*;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_AUTO_TURN;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Utils.AdvancedPidController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagDetector {
    private static final double FORWARD_P = 0;
    private static final double FORWARD_I = 0;
    private static final double FORWARD_D = 0;
    private static final double FORWARD_MAX_I = 0;
    private static final double STRAFE_P = 0;
    private static final double STRAFE_I = 0;
    private static final double STRAFE_D = 0;
    private static final double STRAFE_MAX_I = 0;
    private static final double TURN_P = 0;
    private static final double TURN_I = 0;

    private static final double TURN_D = 0;
    private static final double TURN_MAX_I = 0;
    private static final double FORWARD_MULTIPLIER = 0;
    private static final double STRAFE_MULTIPLIER = 0;
    private static final double TURN_MULTIPLIER = 0;
    public static int DECIMATION = 3;
    public static int CAMERA_EXPOSURE = 4;
    public static int CAMERA_GAIN = 100;

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection targetTag = null;     // Used to hold the data for a detected AprilTag
    public AdvancedPidController speedPid = new AdvancedPidController(FORWARD_P, FORWARD_I, FORWARD_D, FORWARD_MAX_I, "Speed Pid");
    public AdvancedPidController strafePid = new AdvancedPidController(STRAFE_P, STRAFE_I, STRAFE_D, STRAFE_MAX_I, "Strafe Pid");
    public AdvancedPidController turnPid = new AdvancedPidController(TURN_P, TURN_I, TURN_D, TURN_MAX_I, "Turn Pid");

    private HardwareMap hardwareMap;
    private LinearOpMode opMode;
    private boolean targetFound = false;    // Set to true when an AprilTag target is detected
    private double forward = 0;        // Desired forward power/speed (-1 to +1)
    private double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    private double turn = 0;        // Desired turning power/speed (-1 to +1)
    private double cameraOffsetX_inch = 0;
    private double cameraOffsetY_inch = 0;

    public AprilTagDetector(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        initAprilTag();
        setManualExposure(CAMERA_EXPOSURE, CAMERA_GAIN);  // Use low exposure time to reduce motion blur
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor
                .Builder()
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(DECIMATION);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(800, 448))
                .build();
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20);
            }
        }

        // Set camera controls unless we are stopping.
        if (!opMode.isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                opMode.sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            opMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            opMode.sleep(20);
        }
    }

    public AprilTagDetection getTargetAprilTag(int desiredTagId) {
        AprilTagDetection targetTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            //  Check to see if we want to track towards this tag.
            if ((detection.id == desiredTagId)) {
                targetTag = detection;
                break;
            }
        }
        return targetTag;
    }

    public AprilTagDetection getTargetAprilTagSeries(int desiredTagSeries) {
        AprilTagDetection targetTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((detection.id % 3 == desiredTagSeries)) {
                    targetTag = detection;
                    break;
                }
            }
        }
        return targetTag;
    }

    /**
     * @param robotHeading_rad Field-Oriented Heading
     * @return Pose2d of the drivetrain power.
     */
    public Pose2d drive(AprilTagDetection targetTag, Pose2d desiredRelativePose, double robotHeading_rad) {
        if (targetTag == null) {
            return null;
        }

        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double xError = targetTag.ftcPose.x - desiredRelativePose.getX();
        double yError = targetTag.ftcPose.y - desiredRelativePose.getY();
        double headingError = fixHeadingDifference(robotHeading_rad - desiredRelativePose.getHeading());

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        forward = Range.clip(FORWARD_MULTIPLIER * speedPid.calculate(yError), -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        strafe = Range.clip(STRAFE_MULTIPLIER * strafePid.calculate(xError), -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
        turn = Range.clip(TURN_MULTIPLIER * turnPid.calculate(headingError), -MAX_AUTO_TURN, MAX_AUTO_TURN);

        return new Pose2d(forward, strafe, turn);
    }

    public Pose2d calculateError(double desiredDistance, double desiredHeading_rad, double robotHeading_rad) {
        return new Pose2d(
                targetTag.ftcPose.x,
                targetTag.ftcPose.y - desiredDistance,
                robotHeading_rad - desiredHeading_rad
        );
    }

    public double fixHeadingDifference(double heading_rad) {
        while (heading_rad > Math.PI) {
            heading_rad -= 2 * Math.PI;
        }

        while (heading_rad < -Math.PI) {
            heading_rad += 2 * Math.PI;
        }

        return heading_rad;
    }

    public void setCameraOffsets(double offsetX, double offsetY) {
        this.cameraOffsetX_inch = offsetX;
        this.cameraOffsetY_inch = offsetY;
    }

    public Pose2d calculateRobotPose(AprilTagDetection aprilTagDetection, double robotHeading_rad) {
        if (aprilTagDetection == null) {
            return null;
        }

        double currentHeading_deg = Math.toDegrees(robotHeading_rad);
        double theta = Math.toRadians(aprilTagDetection.ftcPose.bearing + currentHeading_deg + 180);

        double x = Math.cos(theta) * aprilTagDetection.ftcPose.range + aprilTagDetection.metadata.fieldPosition.get(0);
        double y = Math.sin(theta) * aprilTagDetection.ftcPose.range + aprilTagDetection.metadata.fieldPosition.get(1);

        if (cameraOffsetX_inch != 0 || cameraOffsetY_inch != 0) {
            Rotation2d currentRotation = new Rotation2d(robotHeading_rad);
            x -= cameraOffsetX_inch * currentRotation.getCos() - cameraOffsetY_inch * currentRotation.getSin();
            y -= cameraOffsetY_inch * currentRotation.getCos() + cameraOffsetX_inch * currentRotation.getSin();
        }

        return new Pose2d(
                x,
                y,
                Math.toRadians(currentHeading_deg)
        );
    }
}
