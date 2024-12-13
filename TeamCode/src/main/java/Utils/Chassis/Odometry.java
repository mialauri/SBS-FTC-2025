package org.firstinspires.ftc.teamcode.Utils.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Utils.KodiakMath;

@Config
public class Odometry {
    /**
     * For H-Bot
     * CAMERA_INSTALLING_POSITION_X = 5.7;
     * CAMERA_INSTALLING_POSITION_Y = 0.3;
     */
    public static double CAMERA_INSTALLING_POSITION_X = 3.71;
    public static double CAMERA_INSTALLING_POSITION_Y = 0;

    public static Rotation2d CAMERA_INIT_HEADING;

    public static double xPoseOffset_in = CAMERA_INSTALLING_POSITION_X;
    public static double yPoseOffset_in = CAMERA_INSTALLING_POSITION_Y;
    public static double headingOffset_rad = 0;

    private static boolean CAMERA_IS_RUNNING = false;

    public Odometry(OpMode opMode) {
        System.loadLibrary("t265");
        opMode.msStuckDetectStop = 3000;
    }

    public void start(double cameraInitHeading) {
        if (!CAMERA_IS_RUNNING) {
            start();
            CAMERA_INIT_HEADING = new Rotation2d(cameraInitHeading);
            CAMERA_IS_RUNNING = true;
        }

        setPoseEstimate(new Pose2d(0, 0, new Rotation2d(0)));
    }

    public void close() {
        if (CAMERA_IS_RUNNING) {
            stop();
            CAMERA_IS_RUNNING = false;
        }
    }

    private native void start();

    private native void stop();

    private native Pose2d getPoseMeters();

    private native Pose2d getVelocity();

    public void waitForInit() {
        getPoseMeters();
    }

    /**
     * @return Robots position in inches.
     */
    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseEstimate() {
        Pose2d rawPose = getPoseMeters();

        double adjustedX = KodiakMath.toInches(rawPose.getX())
                - rawPose.getRotation().getCos() * CAMERA_INSTALLING_POSITION_X
                - rawPose.getRotation().getSin() * CAMERA_INSTALLING_POSITION_Y;

        double adjustedY = KodiakMath.toInches(rawPose.getY())
                - rawPose.getRotation().getSin() * CAMERA_INSTALLING_POSITION_X
                - rawPose.getRotation().getCos() * CAMERA_INSTALLING_POSITION_Y;

        double x = CAMERA_INIT_HEADING.getCos() * adjustedX - CAMERA_INIT_HEADING.getSin() * adjustedY + xPoseOffset_in;
        double y = CAMERA_INIT_HEADING.getSin() * adjustedX + CAMERA_INIT_HEADING.getCos() * adjustedY + yPoseOffset_in;
        double h = rawPose.getHeading() + CAMERA_INIT_HEADING.getRadians() + headingOffset_rad;

        return new com.acmerobotics.roadrunner.geometry.Pose2d(x, y, h);
    }

    /**
     * @return Robots velocity in inches per second.
     */
    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseVelocity() {
        Pose2d currentPose = getVelocity();
        return new com.acmerobotics.roadrunner.geometry.Pose2d(
                -KodiakMath.toInches(currentPose.getX()),
                -KodiakMath.toInches(currentPose.getY()),
                currentPose.getHeading()
        );
    }

    public void setPoseEstimate(Pose2d pose2d) {
        com.acmerobotics.roadrunner.geometry.Pose2d currentPose = getPoseEstimate();

        xPoseOffset_in = pose2d.getX() - (currentPose.getX() - xPoseOffset_in);

        yPoseOffset_in = pose2d.getY() - (currentPose.getY() - yPoseOffset_in);

        headingOffset_rad = pose2d.getHeading() - (currentPose.getHeading() - headingOffset_rad);
    }

    public void setPoseEstimate(double x, double y, double heading) {
        com.acmerobotics.roadrunner.geometry.Pose2d currentPose = getPoseEstimate();

        xPoseOffset_in = x - (currentPose.getX() - xPoseOffset_in);

        yPoseOffset_in = y - (currentPose.getY() - yPoseOffset_in);

        headingOffset_rad = heading - (currentPose.getHeading() - headingOffset_rad);
    }
}
