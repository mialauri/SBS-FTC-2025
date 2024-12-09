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

    private enum VertArmStatus {
        DOWN, UP_BAR, UP_BASKET
    }
    boolean InClawOpen = true;
    boolean hasSample = false;
    public static double CLAW_CLOSE = 0.0;
    public static double CLAW_OPEN = 1.0;

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

        boolean isPressingStart = false;
        boolean isPressingBack = false;
        boolean isPressingLeftStickButton = false;
        boolean isPressingRightStickButton = false;
        boolean isPressingLeftBumper = false;
        boolean isPressingRightBumper = false;
        boolean isPressingLeftTrigger = false;
        boolean isPressingRightTrigger = false;
        boolean isPressingDpadUp = false;
        boolean isPressingDpadDown = false;
        boolean isPressingDpadLeft = false;
        boolean isPressingDpadRight = false;
        boolean isPressingA = false;
        boolean isPressingB = false;
        boolean isPressingX = false;
        boolean isPressingY = false;
        boolean hasSample = false;



        waitForStart();

        while (opModeIsActive()) {

            telemetry.update();

            private  void openClaw() {
                inClaw.setPosition(CLAW_OPEN);
                hasSample = false;
            }

            if (gamepad1.a && !pressA) {
                pressA = true;
            }
            else if (!gamepad1.a && pressA) {
                pressA = false;
            }



            if (gamepad1.b && !pressB && !hasSample) {
                pressB = true;
                if(inClaw.getPosition() == IN_CLAW_CLOSE){
                    inClaw.setPosition(IN_CLAW_OPEN);
                } else if (inClaw.getPosition() == IN_CLAW_OPEN) {
                    inClaw.setPosition(IN_CLAW_CLOSE);
                }
            }
            else if (!gamepad1.b && pressB) {
                pressB = false;
            }

            if (gamepad1.b && !pressB && hasSample) {
                pressB = true;
            }
            else if (!gamepad1.b && pressB) {
                pressB = false;
            }

            if (gamepad1.a && !pressA) {
                pressA = true;
            }
            else if (!gamepad1.a && pressA) {
                pressA = false;
            }


            if (gamepad1.a && !pressA) {
                pressA = true;
            }
            else if (!gamepad1.a && pressA) {
                pressA = false;
            }

            private void closeClaw() {
                inClaw.setPosition(CLAW_CLOSE);
                hasSample = true;
            }


        }
    }
}
