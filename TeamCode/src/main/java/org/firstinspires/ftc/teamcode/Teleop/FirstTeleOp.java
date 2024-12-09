package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Arm.SlideVerticalArm;
import org.firstinspires.ftc.teamcode.Arm.TubeDriver;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "Teleop")
public class FirstTeleOp extends LinearOpMode {
    DcMotorEx lb, lf, rb, rf, intake, vertical1, vertical2;
    Servo inClaw, depClaw, inWrist, depWrist, transfer;
    ColorSensor transColor, inColor;
    IMU imu;
    public static double IN_CLAW_CLOSE = 0.0;
    public static double IN_CLAW_OPEN = 1.0;
    public static double IN_WRIST_DEPOSIT = 0.0;
    public static double IN_WRIST_INTAKE = 1.0;
    public static double OUT_CLAW_CLOSE = 0.0;
    public static double OUT_CLAW_OPEN = 1.0;
    public static double OUT_WRIST_SCORE = 0.0;
    public static double OUT_WRIST_RECIEVE = 1.0;
    public static double DOWN = 0.0;
    public static double UP_BAR = 1.0;
    public static double UP_BASKET = 0.0;
    public static double INTAKE_POWER = 0.0;
    public static double TARGET_INCHES = 0.0;

    boolean pressA = false;
    boolean pressB = false;
    boolean pressX = false;
    boolean pressY = false;
    boolean pressRTRIGGER = false;
    boolean pressLTRIGGER = false;
    boolean pressRBUMPER = false;
    boolean pressLBUMBPER = false;
    boolean pressDPADUP = false;
    boolean pressDPADDOWN = false;
    boolean pressDPADRIGHT = false;
    boolean pressDPADLEFT = false;

    public SlideVerticalArm slideVerticalArm;

    public FirstTeleOp(DcMotorEx vertical1, DcMotorEx vertical2) {
        slideVerticalArm = new SlideVerticalArm(vertical1, vertical2);
        slideVerticalArm.resetPidValues();
    }
    VertArmStatus vertArmStatus = VertArmStatus.DOWN;
    private enum VertArmStatus {
        DOWN, UP_BAR, UP_BASKET
    }
    boolean InClawOpen = true;
    boolean hasSample = false;

    @Override
    public void runOpMode() throws InterruptedException {
        lb       = hardwareMap.get(DcMotorEx .class, "lb");
        lf       = hardwareMap.get(DcMotorEx.class, "lf");
        rb       = hardwareMap.get(DcMotorEx.class, "rb");
        rf       = hardwareMap.get(DcMotorEx.class, "rf");
        intake  = hardwareMap.get(DcMotorEx.class, "intake");
        inClaw   = hardwareMap.get(Servo.class, "inClaw");
        depClaw  = hardwareMap.get(Servo.class, "depClaw");
        inWrist  = hardwareMap.get(Servo.class, "inWrist");
        depWrist = hardwareMap.get(Servo.class, "depWrist");
        transfer = hardwareMap.get(Servo.class, "transfer");
        imu      = hardwareMap.get(IMU.class, "imu");

        lb         = hardwareMap.get(DcMotorEx .class, "lb");
        lf         = hardwareMap.get(DcMotorEx.class, "lf");
        rb         = hardwareMap.get(DcMotorEx.class, "rb");
        rf         = hardwareMap.get(DcMotorEx.class, "rf");
        intake     = hardwareMap.get(DcMotorEx.class, "intake");
        vertical1  = hardwareMap.get(DcMotorEx.class, "vertical1");
        vertical2  = hardwareMap.get(DcMotorEx.class, "vertical2");
        inClaw     = hardwareMap.get(Servo.class, "inClaw");
        depClaw    = hardwareMap.get(Servo.class, "depClaw");
        inWrist    = hardwareMap.get(Servo.class, "inWrist");
        depWrist   = hardwareMap.get(Servo.class, "depWrist");
        transfer   = hardwareMap.get(Servo.class, "transfer");
        imu        = hardwareMap.get(IMU.class, "imu");

        vertical1  = hardwareMap.get(DcMotorEx.class, "vertical1");
        vertical2 = hardwareMap.get(DcMotorEx.class, "vertical2");
        TubeDriver.initRotationMotors(vertical1, vertical2);

        slideVerticalArm = new SlideVerticalArm(vertical1, vertical2);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Has Sample: ", hasSample);
            telemetry.addData("Arm Status: ", vertArmStatus);
            telemetry.update();

            //in claw
            if (gamepad1.x && !pressX && !hasSample) {
                pressX = true;
                if(inClaw.getPosition() == IN_CLAW_CLOSE){
                    inClaw.setPosition(IN_CLAW_OPEN);
                } else if (inClaw.getPosition() == IN_CLAW_OPEN) {
                    inClaw.setPosition(IN_CLAW_CLOSE);
                }
            }
            else if (!gamepad1.x && pressX) {
                pressX = false;
            }

            //dep claw
            if (gamepad1.x && !pressX && hasSample) {
                pressX = true;
                if(depClaw.getPosition() == OUT_CLAW_CLOSE){
                    depClaw.setPosition(OUT_CLAW_OPEN);
                } else if (depClaw.getPosition() == OUT_CLAW_OPEN) {
                    depClaw.setPosition(OUT_CLAW_CLOSE);
                }
            }
            else if (!gamepad1.x && pressX) {
                pressX = false;
            }

            //in wrist
            if (gamepad1.b && !pressB && !hasSample) {
                pressB = true;
                hasSample = true;
                if(inWrist.getPosition() == IN_WRIST_DEPOSIT){
                    inWrist.setPosition(IN_WRIST_INTAKE);
                } else if (inWrist.getPosition() == IN_WRIST_INTAKE) {
                    inWrist.setPosition(IN_WRIST_DEPOSIT);
                }
            }
            else if (!gamepad1.b && pressB) {
                pressB = false;
            }

            //dep wrist
            if (gamepad1.b && !pressB && hasSample) {
                pressB = true;
                hasSample = false;
                if(depWrist.getPosition() == OUT_WRIST_SCORE){
                    depWrist.setPosition(OUT_WRIST_RECIEVE);
                } else if (depWrist.getPosition() == OUT_WRIST_RECIEVE) {
                    depWrist.setPosition(OUT_WRIST_SCORE);
                }
            }
            else if (!gamepad1.b && pressB) {
                pressX = false;
            }

            //up
            if (gamepad1.y && !pressY) {
                pressY = true;
                switch (vertArmStatus) {
                    case DOWN:
                        slideVerticalArm.setToAngleDegrees(UP_BAR);
                        vertArmStatus = VertArmStatus.UP_BAR;
                    case UP_BAR:
                        slideVerticalArm.setToAngleDegrees(UP_BASKET);
                        vertArmStatus = VertArmStatus.UP_BASKET;
                    default:
                        slideVerticalArm.setToAngleDegrees(DOWN);
                        vertArmStatus = VertArmStatus.DOWN;
                }
            }
            else if (!gamepad1.y && pressY) {
                pressY = false;
            }

            //down
            if (gamepad1.a && !pressA) {
                pressA = true;
                switch (vertArmStatus) {
                    case UP_BASKET:
                        slideVerticalArm.setToAngleDegrees(UP_BAR);
                        vertArmStatus = VertArmStatus.UP_BAR;
                    case UP_BAR:
                        slideVerticalArm.setToAngleDegrees(DOWN);
                        vertArmStatus = VertArmStatus.DOWN;
                    default:
                        slideVerticalArm.setToAngleDegrees(UP_BASKET);
                        vertArmStatus = VertArmStatus.UP_BASKET;
                }
            }
            else if (!gamepad1.a && pressA) {
                pressA = false;
            }

            // Intake Slide -->

            // In (Right Trigger)
            if (gamepad1.right_trigger > 1) {
                TARGET_INCHES = 0;
                intake.setPower(getArmPower());
            }
            else {
                intake.setPower(0);
            }

            // Out (Left Trigger)
            if (gamepad1.left_trigger > 1) {
                TARGET_INCHES = 12; //TODO CHECK WHAT THE MAX REACH IS IN INCHES
                intake.setPower(getArmPower());
            }
            else {
                intake.setPower(0);
            }


        }
    }
    public double getArmPower() {
        double pos = intake.getCurrentPosition();
        double angle_deg = (360 / 751.8) * pos; //TODO CHECK TICKS PER REV ON MOTOR
        return pidController.calculate(angle_deg, TARGET_INCHES);
    }

    public void initializeEverything() {

    }
}