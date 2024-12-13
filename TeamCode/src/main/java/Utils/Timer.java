package org.firstinspires.ftc.teamcode.Utils;

public class Timer {
    long endTimeMillis;

    public Timer(long startTimeMillis, long waitTimeMillis) {
        endTimeMillis = startTimeMillis + waitTimeMillis;
    }

    public boolean reachedTime() {
        return System.currentTimeMillis() > endTimeMillis;
    }
}
