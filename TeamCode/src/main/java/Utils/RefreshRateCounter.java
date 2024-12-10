package org.firstinspires.ftc.teamcode.Utils;

public class RefreshRateCounter {
    long lastInputTime;
    int lastRefreshRate = 0;
    double lastInput = 0;

    public RefreshRateCounter() {
        lastInputTime = System.currentTimeMillis();
    }

    /**
     * Calculate the refresh rate.
     *
     * @param newInput Input value
     * @return The newest updated refresh rate
     */
    public int update(double newInput) {
        if (newInput == lastInput)
            return lastRefreshRate;

        long currentTime = System.currentTimeMillis();
        lastRefreshRate = (int) Math.round(1000.0 / (currentTime - lastInputTime));
        lastInput = newInput;
        lastInputTime = currentTime;

        return lastRefreshRate;
    }
}