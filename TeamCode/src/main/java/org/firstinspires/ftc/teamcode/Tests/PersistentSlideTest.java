package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.acmerobotics.dashboard.config.Config;

@TeleOp(name = "PersistentSlideTest")
@Config
public class PersistentSlideTest extends LinearOpMode {
    DcMotorEx motor;
    double pos;
    public static double SPEED = 0;
    public static int TARGETUPPOSITION = 200;
    public static int TARGETDOWNPOSITION = -20;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //if you leave in this mode it won't move

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //need this line when you use the above line
        // or it won't move

        waitForStart();

        while (opModeIsActive()) {

            //double diameterBig = 28.8; //mm
            //double diameterSmall = 7; //mm

            double speedAdd = 0;
            boolean pressY = false;
            boolean pressA = false;
            boolean pressB = false;
            boolean pressX = false;
            boolean pressDpadUp = false;
            boolean pressDpadDown = false;
            double deg = 0;
            double power = gamepad1.right_stick_y;
            double stopSpeedMax = 0.9;


            while (opModeIsActive()) {//TODO REVERSE EVERYTHING POS SET OPPOSITE AND TELEMETRY AND BUTTONS SHOW OPPOSITE
                pos = motor.getCurrentPosition();
                deg = (360 / 751.8) * pos * (28.8 / 7);


                while (gamepad1.y && pos < TARGETUPPOSITION) {
                    motor.setTargetPosition(TARGETUPPOSITION);
                    motor.setPower(SPEED);
                    pos = motor.getCurrentPosition();
                }
                motor.setPower(0);


                while (gamepad1.a && pos > TARGETDOWNPOSITION) {
                    motor.setTargetPosition(TARGETDOWNPOSITION);
                    motor.setPower(-SPEED);
                    pos = motor.getCurrentPosition();
                }
                motor.setPower(0);

                if(gamepad1.dpad_down && SPEED > 0 && !pressDpadDown) {
                    pressDpadDown = true;
                    SPEED-=0.1;
                } else if (!gamepad1.dpad_down && pressDpadDown) {
                    pressDpadDown = false;
                }

                if(gamepad1.dpad_up && SPEED < 1 && !pressDpadUp) {
                    pressDpadUp = true;
                    SPEED+=0.1;
                } else if (!gamepad1.dpad_up && pressDpadUp) {
                    pressDpadUp = false;
                }

                telemetry.addData("motor SPEED: ", SPEED);
                telemetry.addData("Pos: ", pos);
                telemetry.addData("Degrees: ", deg);
                telemetry.update();
            }
        }
    }
}





//slide test : max pos 3512
// degree = 6919.0925

// min pos -29 , degree -57
