package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "Teleop")
public class FirstTeleOp extends LinearOpMode {
    DcMotorEx lb, lf, rb, rf, intake, vertical1, vertical2;
    Servo inClaw, depClaw, inWrist, depWrist, transfer;
    ColorSensor transColor, inColor;
    IMU imu;

    arm
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
    public static double INTAKE_SLIDE_FAST = 0.0;
    public static double INTAKE_SLIDE_SLOW = 0.0;

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
        vertical1  = hardwareMap.get(DcMotorEx.class, "vertical1");
        vertical2 = hardwareMap.get(DcMotorEx.class, "vertical2");
        inClaw   = hardwareMap.get(Servo.class, "inClaw");
        depClaw  = hardwareMap.get(Servo.class, "depClaw");
        inWrist  = hardwareMap.get(Servo.class, "inWrist");
        depWrist = hardwareMap.get(Servo.class, "depWrist");
        transfer = hardwareMap.get(Servo.class, "transfer");
        imu      = hardwareMap.get(IMU.class, "imu");



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
                        vertical1.setTargetPosition(UP_BAR);
                        vertical2.setTargetPosition(UP_BAR);
                        vertArmStatus = VertArmStatus.UP_BAR;
                    case UP_BAR:
                        vertical1.setTargetPosition(UP_BASKET);
                        vertical2.setTargetPosition(UP_BASKET);
                        vertArmStatus = VertArmStatus.UP_BASKET;
                    default:
                        vertical1.setTargetPosition(DOWN);
                        vertical2.setTargetPosition(DOWN);
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
                        vertical1.setTargetPosition(UP_BAR);
                        vertical2.setTargetPosition(UP_BAR);
                        vertArmStatus = VertArmStatus.UP_BAR;
                    case UP_BAR:
                        vertical1.setTargetPosition(DOWN);
                        vertical2.setTargetPosition(DOWN);
                        vertArmStatus = VertArmStatus.DOWN;
                    default:
                        vertical1.setTargetPosition(UP_BASKET);
                        vertical2.setTargetPosition(UP_BASKET);
                        vertArmStatus = VertArmStatus.UP_BASKET;
                }
            }
            else if (!gamepad1.a && pressA) {
                pressA = false;
            }
        }
    }
}

