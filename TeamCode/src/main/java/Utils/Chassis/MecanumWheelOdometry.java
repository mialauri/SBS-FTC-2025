package org.firstinspires.ftc.teamcode.Utils.Chassis;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.WHEEL_RADIUS;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumWheelOdometry {
    private DcMotorEx lf;
    private DcMotorEx rf;
    private DcMotorEx lb;
    private DcMotorEx rb;
    private BNO055IMU imu;

    private MecanumDriveOdometry wheelOdometry;

    public MecanumWheelOdometry(DcMotorEx lf, DcMotorEx rf, DcMotorEx lb, DcMotorEx rb) {
        this.lf = lf;
        this.rf = rf;
        this.lb = lb;
        this.rb = rb;

        ChassisDriver.resetWheelEncoders(lf, rf, lb, rb);
    }

    public MecanumWheelOdometry(DcMotorEx lf, DcMotorEx rf, DcMotorEx lb, DcMotorEx rb, BNO055IMU imu) {
        this.lf = lf;
        this.rf = rf;
        this.lb = lb;
        this.rb = rb;
        this.imu = imu;

        wheelOdometry = new MecanumDriveOdometry(KodiakDriver.KINEMATICS, new Rotation2d(), new Pose2d());

        ChassisDriver.resetWheelEncoders(lf, rf, lb, rb);
    }

    public void updatePoseEstimate() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                lf.getVelocity(), rf.getVelocity(),
                lb.getVelocity(), rb.getVelocity()
        );
        wheelOdometry.updateWithTime(System.currentTimeMillis() / 1000.0, getGyroHeading(), wheelSpeeds);
    }

    public Pose2d getPoseMeters() {
        return wheelOdometry.getPoseMeters();
    }

    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseEstimate() {
        Pose2d currentPose = wheelOdometry.getPoseMeters();

        return new com.acmerobotics.roadrunner.geometry.Pose2d(
                currentPose.getX(),
                currentPose.getY(),
                currentPose.getHeading()
        );
    }

    public com.acmerobotics.roadrunner.geometry.Pose2d getPoseVelocity() {
        double vlf = lf.getVelocity();
        double vrf = rf.getVelocity();
        double vlb = lf.getVelocity();
        double vrb = rb.getVelocity();

        double vF = (vlf + vrf + vlb + vrb) / 4;
        double vS = (-vlf + vrf + vlb - vrb) / 4;
        double w = (-vlf + vrf - vlb + vrb) / (4 * 2 * KodiakDriver.trackDiameter);
        return new com.acmerobotics.roadrunner.geometry.Pose2d(vF, vS, w);
    }

    public Rotation2d getGyroHeading() {
        return new Rotation2d(imu.getAngularOrientation().firstAngle);
    }

    public void setPoseEstimate(com.acmerobotics.roadrunner.geometry.Pose2d pose) {
        wheelOdometry.resetPosition(
                new Pose2d(
                        pose.getX(),
                        pose.getY(),
                        new Rotation2d(pose.getHeading())
                ),
                new Rotation2d(imu.getAngularOrientation().firstAngle)
        );
    }

    public static double encoderTicksToMeters(double ticks) {
        return (WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV) * 0.0254;
    }
}

