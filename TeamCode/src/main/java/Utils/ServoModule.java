package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoModule {
    Servo servo;
    private double targetAngle;

    private final double servoOffset;
    private final double valuePerRotation;

    /**
     * Constructor of a ServoModule.
     *
     * @param servo Servo from hardwareMap
     * @param offset Offset angle of 0 degree position
     * @param valuePerRotation Value per rotation. 360-degree servo should be set to 1.
     */
    public ServoModule(Servo servo, double offset, double valuePerRotation) {
        this.servo = servo;
        this.servoOffset = offset;
        this.valuePerRotation = valuePerRotation;
    }

    /**
     * Set to a specific angle of rotation.
     *
     * @param angle A positive angle in degrees
     */
    public void setToAngle(double angle) {
        angle = Math.max(0, angle);
        targetAngle = angle;

        servo.setPosition(((angle + servoOffset) / 360) * valuePerRotation);
    }

    public double getTargetAngle() {
        return targetAngle;
    }
}
