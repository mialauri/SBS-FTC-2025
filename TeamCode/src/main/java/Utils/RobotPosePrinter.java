package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;

//import org.firstinspires.ftc.teamcode.RoadRunner.util.DashboardUtil;

import java.util.LinkedList;

public class RobotPosePrinter {
    public static int POSE_HISTORY_LIMIT = 100;
    private LinkedList<Pose2d> poseHistory;
    private FtcDashboard dashboard;

    public RobotPosePrinter() {
        initializeDashboard();
    }

    public void initializeDashboard() {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        poseHistory = new LinkedList<>();
    }

    public void printCurrentRobotPosition(Pose2d currentPose) {
        poseHistory.add(currentPose);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        fieldOverlay.setStroke("#3F51B5");
     //   org.firstinspires.ftc.teamcode.RoadRunner.util.DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        dashboard.sendTelemetryPacket(packet);
    }
}
