package org.firstinspires.ftc.teamcode.Utils.Chassis;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

public class SwerveModule {
    private final Motor driveMotor;
    private final Servo turningServo;

    private final Motor.Encoder driveEncoder;
    private final Motor.Encoder turningEncoder;

    private final PIDFController turningPidController;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffset;

    public SwerveModule(Motor driveMotor, Servo turningServo, Motor.Encoder driveEncoder,
                        Motor.Encoder turningEncoder, PIDFController turningPidController,
                        boolean absoluteEncoderReversed, double absoluteEncoderOffset) {
        this.driveMotor = driveMotor;
        this.turningServo = turningServo;
        this.driveEncoder = driveEncoder;
        this.turningEncoder = turningEncoder;
        this.turningPidController = turningPidController;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoderOffset = absoluteEncoderOffset;

    }
}
