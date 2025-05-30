package org.firstinspires.ftc.teamcode.blucru.common.util;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;

public class Globals {
    public static HardwareMap hwMap; // global reference to current hwmap
    public static Telemetry tele; // global reference to current telemetry

    public static double reflect = 1;

    // default alliance is red, but will be changed before auto starts
    public static Alliance alliance = Alliance.RED;
    public static ElapsedTime runtime;

    public static double voltage = 13.0;

    public static void setAlliance(Alliance alliance) {
        Globals.alliance = alliance;
        Globals.reflect = alliance == Alliance.RED ? 1 : -1;
    }

    public static void flipAlliance() {
        setAlliance(Globals.alliance.flip());
    }

    public static Pose2d mapPose(double x, double y, double headingDeg) {
        x = x*reflect;
        y = y*reflect;
        if(reflect < 0) {
            headingDeg += 180;
        }

        return new Pose2d(x, y, Angle.norm(Math.toRadians(headingDeg)));
    }

    public static Pose2d mapPose(Pose2d pose) {
        return mapPose(pose.getX(), pose.getY(), pose.getHeading());
    }

    public static Vector2d mapPoint(Vector2d point) {
        double x = point.getX()*reflect;
        double y = point.getY()*reflect;

        return new Vector2d(x, y);
    }

    public static Pose2d unMapPose(Pose2d pose) {
        double x = pose.getX() * reflect;
        double y = pose.getY() * reflect;
        double heading = alliance == Alliance.RED ?
                pose.getHeading() :
                Angle.norm(pose.getHeading() + Math.PI);

        return new Pose2d(x, y, heading);
    }

    public static void setVoltage(double voltage) {
        Globals.voltage = voltage;
        Log.i("Globals", "set voltage to " + voltage);
    }

    public static double correctPower(double power) {
        return power * 12.0 / Globals.voltage;
    }

    public static void runtimeTelemetry() {
        tele.addData("Runtime", Globals.runtime.seconds());
    }

    public static void drawPose(Pose2d pose) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay()
                .setStroke("#1d38cf");
        DashboardUtil.drawRobot(fieldOverlay, pose);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public static double time() {
        return runtime.milliseconds();
    }

    public static double timeSince(double time) {
        return runtime.milliseconds() - time;
    }
}
