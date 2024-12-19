package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SlideTest")
public class SlideTest extends LinearOpMode {
    DcMotorEx motor;
    double pos;

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

            double speed = 0;
            double speedAdd = 0;
            boolean pressY = false;
            boolean pressA = false;
            boolean pressB = false;
            boolean pressX = false;
            double deg = 0;
            int targetUpPosition = 200;
            int targetDownPosition = -20;
            double power = gamepad1.right_stick_y;
            double stopSpeedMax = 0.9;


            while (opModeIsActive()) {
                pos = motor.getCurrentPosition();
                deg = (360 / 751.8) * pos * (28.8 / 7);
                speed = 0;

               /* if (gamepad1.b && !pressB) { //if pressing b and var is false
                    pressB = true;
                    speed += 0.1;
                } else if (!gamepad1.b && pressB) {
                    pressB = false;
                }

                if (gamepad1.x && !pressX) {
                    pressX = true;
                    speed -= 0.1;
                } else if (!gamepad1.x && pressX) {
                    pressX = false;
                }

                */

             /*   if(speed >= 0.9 || speed <= -0.3 ){
                    motor.setPower(0);
                    motor.setMode(DcMotor.RunMode.RESET_ENCODERS);
                }

              */

                if (gamepad1.y && pos < targetUpPosition && !pressY) {//if y is pressed slide go up
                    speed = 0.5;
                    speed -= power;
                    pressY = true;
                    motor.setPower(speed);
                    motor.setTargetPosition(targetUpPosition);
                } else if (pos >= targetUpPosition && pressA) {
                    motor.setPower(0);
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pressY = false;
                }


                if (gamepad1.a && pos > targetDownPosition && !pressA) {//if a is pressed slide go down
                    speed = -0.5;
                    speed -= power;
                    pressA = true;
                    motor.setPower(speed);
                } else if (pos <= targetDownPosition && pressA) {
                    motor.setPower(0);
                    motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pressA = false;
                }
                if (gamepad1.b && !pressB) {
                    motor.setPower(0);
                    pressA = true;}

                telemetry.addData("motor speed", speed);
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
