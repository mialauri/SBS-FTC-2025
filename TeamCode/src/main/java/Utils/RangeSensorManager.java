package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RangeSensorManager {
    private final static int MAX_OUT_OF_THRESHOLD_COUNT = 5;
    private double maxDistanceCM = Double.MAX_VALUE;

    private ModernRoboticsI2cRangeSensor rangeSensor;
    private boolean isUsingOpticalSensor = false;

    private boolean enabledFreshRateCheck = false;
    private int readPerSecond = 0;

    private volatile double currentVelocity = 0;
    private double latestReadCM;

    private boolean isUsingThreshold;
    private double denoisingThresholdValue;
    private double previousThresholdReadCM;
    private int outOfThresholdCounter;

    Thread rangeSensorThread = new Thread(() -> {
        long oneSecondStartTimeMillis = System.currentTimeMillis();
        double previousRead = 0;
        int counter = 0;

        long velocityStartTimeMillis = oneSecondStartTimeMillis;
        double velocityInitialReadCM = 0;

        while (!Thread.currentThread().isInterrupted()) {
            long currentTimeMillis = System.currentTimeMillis();
            double currentReadCM = getDistanceCM();

            /**
             * Get Current Velocity.
             */
            if (currentTimeMillis - velocityStartTimeMillis >= 100) {
                currentVelocity = -1000.0 * (currentReadCM - velocityInitialReadCM) / (currentTimeMillis - velocityStartTimeMillis);
                RobotLog.d("CurrentRead" + currentReadCM + "; VelocityStart" + velocityInitialReadCM + "; Interval " + (currentTimeMillis - velocityStartTimeMillis) + "; CurrentVelocity " + currentVelocity);
                velocityInitialReadCM = currentReadCM;
                velocityStartTimeMillis = currentTimeMillis;
            }

            /**
             * Fresh rate check.
             */
            if (enabledFreshRateCheck) {
                /**
                 * Check if one second reached.
                 */
                if (currentTimeMillis - oneSecondStartTimeMillis >= 1000) {
                    readPerSecond = counter;
                    counter = 0;
                    oneSecondStartTimeMillis = currentTimeMillis;
                }

                /**
                 * Check if Range Value has changed.
                 */
                if (currentReadCM != previousRead) {
                    counter++;
                    previousRead = currentReadCM;
                }
            }
        }
    });

    public RangeSensorManager(ModernRoboticsI2cRangeSensor rangeSensor) {
        this.rangeSensor = rangeSensor;
    }

    public Thread start() {
        rangeSensorThread.start();
        return rangeSensorThread;
    }

    public void interrupt() {
        rangeSensorThread.interrupt();
    }

    public double getVelocity() {
        return currentVelocity;
    }

    public void enableFreshRateCheck() {
        enabledFreshRateCheck = true;
    }

    public void disableFreshRateCheck() {
        enabledFreshRateCheck = false;
    }

    public void useOpticalSensor() {

    }

    public int getFreshRatePerSecond() {
        return readPerSecond;
    }

    public void setDenoisingThresholdValue(double denoisingThresholdValue) {
        this.denoisingThresholdValue = denoisingThresholdValue;
        isUsingThreshold = true;
        previousThresholdReadCM = getDistanceCM();
    }

    public void setMaxDistanceCM(double maxDistanceCM) {
        this.maxDistanceCM = maxDistanceCM;
    }

    public boolean isWithInThreshold(double currentReadCM) {
        if ((Math.abs(currentReadCM - previousThresholdReadCM) <= denoisingThresholdValue && currentReadCM <= maxDistanceCM)) {
            previousThresholdReadCM = currentReadCM;
            outOfThresholdCounter = 0;
            return true;
        } else if (outOfThresholdCounter > MAX_OUT_OF_THRESHOLD_COUNT) {
            previousThresholdReadCM = currentReadCM;
            outOfThresholdCounter = 0;
            return true;
        } else {
            outOfThresholdCounter++;
            return false;
        }
    }

    /**
     * Get value by using threshold and store the values for future uses.
     *
     * @param rawRead Distance in CM.
     * @return Filtered value calculated by the set threshold.
     */
    public double filterWithThreshold(double rawRead) {
        if (Double.isNaN(rawRead))
            return previousThresholdReadCM;
        else if (!isWithInThreshold(rawRead)) {
            return previousThresholdReadCM;
        } else {
            latestReadCM = rawRead;
            return rawRead;
        }
    }

    public double getDistanceCM() {
        double tempRead;

        if (isUsingOpticalSensor) {
            tempRead = getRawOpticalCM();
        } else
            tempRead = getRawCM();

        if (isUsingThreshold) {
            latestReadCM = filterWithThreshold(tempRead);
            return latestReadCM;
        } else {
            latestReadCM = tempRead;
            return tempRead;
        }

    }

    public double getRawCM() {
        return rangeSensor.getDistance(DistanceUnit.CM);
    }

    public double getRawOpticalCM() {
        return rangeSensor.cmOptical();
    }
}
