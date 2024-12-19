package org.firstinspires.ftc.teamcode.Utils.Arm;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorDriver {

    private static DcMotorEx lf, rf, lb, rb;
    public MotorDriver (DcMotorEx lf, DcMotorEx rf, DcMotorEx lb, DcMotorEx rb) {
        MotorDriver.lf = lf;
        MotorDriver.lb = lb;
        MotorDriver.rf = rf;
        MotorDriver.rb = rb;
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setMotorPower (double power) {
        lf.setPower(power);
        lb.setPower(power);
        rf.setPower(power);
        rb.setPower(power);
    }
}
