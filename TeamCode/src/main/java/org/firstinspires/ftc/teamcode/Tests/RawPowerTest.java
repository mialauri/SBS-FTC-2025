package org.firstinspires.ftc.teamcode.Tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Arm.TubeDriver;
import org.firstinspires.ftc.teamcode.Utils.Arm.MotorDriver;

@TeleOp(name = "RawPowerTest")
public class RawPowerTest extends LinearOpMode{
    DcMotorEx lb, lf, rb, rf;

    MotorDriver motorDriver;

    @Override
    public void runOpMode() throws InterruptedException {
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        TubeDriver.initRotationMotors(lf, rf, lb, rb);
        motorDriver = new MotorDriver(lf, rf, lb, rb);

        waitForStart();

        while (opModeIsActive()) {


            motorDriver.setMotorPower(gamepad1.left_stick_y);
            telemetry.addData("Power", gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}

