package org.firstinspires.ftc.teamcode.Utils.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class TubeDriver {
    public static double ROTATION_P = 0.013;
    public static double ROTATION_I = 0.008;    // (0.006, Dec 7, 2023)
    public static double ROTATION_D = 0;
    public static double ROTATION_T = 0.5;
    public static double ROTATION_PID_POWER_LIMIT = 0.5;
    public static double ROTATION_Kx = -0.13;
    public static double ROTATION_Kv = 0.25;
    public static double ROTATION_I_MAX = 0.2;

    public static int ROTATION_OFFSET_TICKS = 1090;
    public static double TICKS_PER_TUBE_ROTATION_2 = 8192;


    public final ArmRotationDriver armRotationDriver;

    /**
     * Slide motors are set to RUN_WITHOUT_ENCODER.
     *
     * @param rotationMotor1  Rotation DcMotorEx with no reversing.
     */
    public TubeDriver(DcMotorEx rotationMotor1, DcMotorEx rotationMotor2) {
        armRotationDriver = new ArmRotationDriver(rotationMotor1, rotationMotor2);
        armRotationDriver.resetPidValues();
    }

    public void update() {
        armRotationDriver.updateMotorPower();
    }

    /**
     * Set the target position of the Slide_Motors in inches.
     * Set the target position of the Rotation-Motor in degrees.
     *
     * @param extensionPosition_ticks The length of the target position in ticks.
     * @param rotationAngle_degrees   The angle of the target position in degrees.
     */
    public void setArmPosition(int extensionPosition_ticks, double rotationAngle_degrees) {
        armRotationDriver.setToAngleDegrees(rotationAngle_degrees);
    }

    public double getCurrentAngleDegrees() {
        return armRotationDriver.getCurrentAngleDegrees();
    }

    public double getTargetAngle() {
        return armRotationDriver.rotationTargetAngle;
    }

    public static void initExtensionMotors(DcMotorEx extensionMotor1, DcMotorEx extensionMotor2) {
        extensionMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        extensionMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extensionMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public static void initRotationMotors(DcMotorEx lf, DcMotorEx lb, DcMotorEx rf, DcMotorEx rb) {
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void stopAllMotors() {
        armRotationDriver.stopMotors();
    }

    public void rotateUp() {
        armRotationDriver.manualControl(-1);
    }
}
