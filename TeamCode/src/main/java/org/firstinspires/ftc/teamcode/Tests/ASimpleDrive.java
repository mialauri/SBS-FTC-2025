package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Utils.Chassis.ChassisDriver;
import org.firstinspires.ftc.teamcode.Utils.Chassis.KodiakDriver;

@TeleOp(group = "Officials")
public class ASimpleDrive extends LinearOpMode {

    public static double FAST_SPEED_MULTIPLIER = 2;
    public static double FAST_TURN_MULTIPLIER = 4;
    public static double SLOW_SPEED_MULTIPLIER = 0.8;
    public static double SLOW_TURN_MULTIPLIER = 1.5;
    boolean IsFastMode = false;
    DcMotorEx lf, rf, lb, rb;
    IMU imu;

    boolean isPressingX = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.clearAll();
        telemetry.addLine("Initializing");
        telemetry.update();

        // initialize motors and sensors
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        ChassisDriver.initializeMotors(lf, rf, lb, rb);

        imu = hardwareMap.get(IMU.class, "imu");
        KodiakDriver.initializeIMU(imu);

        telemetry.clearAll();
        telemetry.addLine("READY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Press X to toggle fast driving");
            telemetry.addData("Is Fast Mode", IsFastMode);
            telemetry.update();


            if (gamepad1.x && !isPressingX) {
                isPressingX = true;
                IsFastMode = !IsFastMode;
            } else if (!gamepad1.x && isPressingX) {
                isPressingX = false;
            }

            if (IsFastMode) {
                ChassisDriver.rawDrive(
                        -gamepad1.left_stick_y * ChassisDriver.FAST_SPEED_MULTIPLIER,
                        -gamepad1.left_stick_x * ChassisDriver.FAST_SPEED_MULTIPLIER,
                        -gamepad1.right_stick_x * ChassisDriver.FAST_TURN_MULTIPLIER
                );
            } else {
                ChassisDriver.rawDrive(
                        -gamepad1.left_stick_y * ChassisDriver.SLOW_SPEED_MULTIPLIER,
                        -gamepad1.left_stick_x * ChassisDriver.SLOW_SPEED_MULTIPLIER,
                        -gamepad1.right_stick_x * ChassisDriver.SLOW_TURN_MULTIPLIER
                );
            }
        }
    }
}
