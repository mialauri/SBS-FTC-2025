package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class PidValuesManager {
    private Gamepad gamepad1;

    private PIDController[] pidControllers;
    private double[][] pidValues;
    private int numberOfGroups;
    private int currentGroup = 0;
    private int currentPidSelection = 0;
    public static double difference;
    public static double toleranceDifference = 0.01;

    private boolean isPressingDpadLeft = false;
    private boolean isPressingDpadRight = false;
    private boolean isPressingDpadUp = false;
    private boolean isPressingDpadDown = false;

    /**
     * @param gamepad1       gampad1 from OpMode.
     * @param numberOfGroups Number of PID Controllers needed.
     * @param difference     Delta of each press on Gamepad.
     */
    public PidValuesManager(Gamepad gamepad1, int numberOfGroups, double difference) {
        this.gamepad1 = gamepad1;
        this.numberOfGroups = numberOfGroups;
        this.difference = difference;

        /**
         * Initialize PID values.
         */
        pidControllers = new PIDController[numberOfGroups];
        pidValues = new double[numberOfGroups][4];
        for (int i = 0; i < numberOfGroups; i++) {
            pidControllers[i] = new PIDController(0, 0, 0);
            pidValues[i] = new double[4];
        }
    }

    public PidValuesManager(int numberOfGroups, double p, double i, double d, double t) {
        this.numberOfGroups = numberOfGroups;

        /**
         * Initialize PID values.
         */
        pidControllers = new PIDController[numberOfGroups];
        for (int j = 0; j < numberOfGroups; j++) {
            pidControllers[j] = new PIDController(p, i, d);
            pidControllers[j].setTolerance(t);
        }
    }

    public double[] getPidGroup(int groupNumber) {
        return pidValues[groupNumber];
    }

    public double calculate(int groupNumber, double value) {
        return pidControllers[groupNumber].calculate(value);
    }

    public void update() {
        if (gamepad1.dpad_up && !isPressingDpadUp) {
            isPressingDpadUp = true;

            if (currentPidSelection == 3)
                pidValues[currentGroup][currentPidSelection] += toleranceDifference;
            else
                pidValues[currentGroup][currentPidSelection] += difference;

            pidValues[currentGroup][currentPidSelection] = roundNumber(pidValues[currentGroup][currentPidSelection]);
            resetPid(currentGroup);
        } else if (!gamepad1.dpad_up && isPressingDpadUp)
            isPressingDpadUp = false;

        if (gamepad1.dpad_down && !isPressingDpadDown) {
            isPressingDpadDown = true;

            if (currentPidSelection == 3)
                pidValues[currentGroup][currentPidSelection] -= toleranceDifference;
            else
                pidValues[currentGroup][currentPidSelection] -= difference;

            pidValues[currentGroup][currentPidSelection] = roundNumber(pidValues[currentGroup][currentPidSelection]);
            resetPid(currentGroup);
        } else if (!gamepad1.dpad_down && isPressingDpadDown)
            isPressingDpadDown = false;

        if (gamepad1.dpad_left && !isPressingDpadLeft) {
            isPressingDpadLeft = true;
            currentGroup = (currentGroup + 1) % numberOfGroups;
        } else if (!gamepad1.dpad_left && isPressingDpadLeft)
            isPressingDpadLeft = false;

        if (gamepad1.dpad_right && !isPressingDpadRight) {
            isPressingDpadRight = true;
            currentPidSelection = (currentPidSelection + 1) % 4;

        } else if (!gamepad1.dpad_right && isPressingDpadRight)
            isPressingDpadRight = false;
    }

    public void setTolerance(int groupNumber, double tolerance) {
        pidValues[groupNumber][3] = tolerance;
        resetPid(groupNumber);
    }

    public void resetPid(int groupNumber) {
        pidControllers[groupNumber].setPID(pidValues[groupNumber][0], pidValues[groupNumber][1], pidValues[groupNumber][2]);
        pidControllers[groupNumber].setTolerance(pidValues[groupNumber][3]);
        pidControllers[groupNumber].reset();
    }

    public void resetAllPids() {
        for (int i = 0; i < numberOfGroups; i++) {
            resetPid(i);
        }
    }

    public String getCurrentSelectionString() {
        return "Group: " + currentGroup + "; Selection (0-3): " + currentPidSelection + ";";
    }

    public String getCurrentPidString() {
        return "P: " + pidValues[currentGroup][0] +
                "; I: " + pidValues[currentGroup][1] +
                "; D: " + pidValues[currentGroup][2] +
                "; T: " + pidValues[currentGroup][3];
    }

    public double roundNumber(double a) {
        return Math.round(a * 100000) / 100000.0;
    }

    public void setPid(int groupNumber, double p, double i, double d, double t) {
        pidValues[groupNumber][0] = p;
        pidValues[groupNumber][1] = i;
        pidValues[groupNumber][2] = d;
        pidValues[groupNumber][3] = t;

        resetPid(groupNumber);
    }
}
