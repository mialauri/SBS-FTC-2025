/*******************************************************************************
 * KodiakDriver.java, 2022/9/26 下午5:15.
 * Copyright (c) 2022 Xuefei (William) Tao. All Rights Reserved.
 ******************************************************************************/

package org.firstinspires.ftc.teamcode.Utils.Chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class KodiakDriver {
    /**
     * Front-rear 0.16/2 = 0.08
     * Left-right 0.1750/2 = 0.0875
     */
    public final static int MAX_ENCODER_VELOCITY = 2700;

    public static final double TICKS_PER_TUBE_INCH = 41;

    public final static double trackDiameter = 0.16;
    private final static Translation2d lfLocation = new Translation2d(0.095, 0.096);
    private final static Translation2d rfLocation = new Translation2d(0.095, -0.096);
    private final static Translation2d lbLocation = new Translation2d(-0.095, 0.096);
    private final static Translation2d rbLocation = new Translation2d(-0.095, -0.096);
    public final static MecanumDriveKinematics KINEMATICS = new MecanumDriveKinematics(lfLocation, rfLocation, lbLocation, rbLocation);

    public static PIDController HEADING_PID_CONTROLLER = new PIDController(2, 0, 0);

    private static BNO055IMU imu;

    public static void initializeIMU(BNO055IMU imu) {
        KodiakDriver.imu = imu;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        KodiakDriver.imu.initialize(parameters);
    }

    public static void initializeIMU(IMU imu) {
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );
        imu.resetYaw();
    }

    public static Telemetry getDashboard() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        return dashboard.getTelemetry();
    }
}
