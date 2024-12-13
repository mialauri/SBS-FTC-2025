package org.firstinspires.ftc.teamcode.Utils.Chassis;

public class MecanumWheelEncodersLocator {
    private final double QUARTER_PI = Math.PI / 4;
    private double x, y, r;
    private double previousLfTicks, previousRfTicks, previousLbTicks, previousRbTicks;
    private double ticksPerInch, ticksPerRevolution;

    public MecanumWheelEncodersLocator(double initialX, double initialY, double initialR, double initialLfTicks, double initialRfTicks, double initialLbTicks, double initialRbTicks, double ticksPerInch, double ticksPerRevolution) {
        this.x = initialX * ticksPerInch;
        this.y = initialY * ticksPerInch;
        this.r = initialR * ticksPerRevolution;
        this.previousLfTicks = initialLfTicks;
        this.previousRfTicks = initialRfTicks;
        this.previousLbTicks = initialLbTicks;
        this.previousRbTicks = initialRbTicks;
        this.ticksPerInch = ticksPerInch;
        this.ticksPerRevolution = ticksPerRevolution;
    }

    public void update(double lfTicks, double rfTicks, double lbTicks, double rbTicks) {
        double deltaLf = lfTicks - previousLfTicks;
        double deltaRf = rfTicks - previousRfTicks;
        double deltaLb = lbTicks - previousLbTicks;
        double deltaRb = rbTicks - previousRbTicks;

        previousLfTicks = lfTicks;
        previousRfTicks = rfTicks;
        previousLbTicks = lbTicks;
        previousRbTicks = rbTicks;

        double dx = deltaLf - deltaRf - deltaLb + deltaRb;
        double dy = (deltaLf + deltaRf + deltaLb + deltaRb);
        r += -deltaLf + deltaRf - deltaLb + deltaRb;

        x += -dy * Math.sin(r) + dx * Math.sin(QUARTER_PI * 2 - getR());
        y += dy * Math.cos(r) + dx * Math.cos(QUARTER_PI * 2 - getR());

    }

    /**
     * @return Value of x in inch.
     */
    public double getX() {
        return x;
    }

    /**
     * @return Value of y in inch.
     */
    public double getY() {
        return y;
    }

    /**
     * @return Value of r in inch.
     */
    public double getR() {
        return (r / ticksPerRevolution) * (2 * Math.PI);
    }
}
