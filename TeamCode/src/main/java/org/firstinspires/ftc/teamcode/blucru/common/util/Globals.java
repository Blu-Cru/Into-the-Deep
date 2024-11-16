package org.firstinspires.ftc.teamcode.blucru.common.util;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Globals {
    // default pose for the robot, will be changed at the end of auto
    public static Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
    public static HardwareMap hwMap; // global reference to current hwmap
    public static Telemetry tele; // global reference to current telemetry

    public static double reflect = -1;

    // default alliance is red, but will be changed before auto starts
    public static Alliance alliance = Alliance.RED;
    public static Side side = Side.RIGHT_SPECIMEN;

    public static ElapsedTime runtime;

    public static double voltage = 13.0;

    public static void startAuto() {
        Globals.runtime = new ElapsedTime();
        Globals.runtime.reset();
    }

//    public static void setAutoStartPose() {
//        if(side == Side.AUDIENCE) Globals.startPose = Globals.mapPose(-36, 62, 90);
//        else Globals.startPose = Globals.mapPose(12, 62, 90);
//    }

    public static void setAlliance(Alliance alliance) {
        Globals.alliance = alliance;
        Globals.reflect = alliance == Alliance.RED ? -1 : 1;
    }

    public static Pose2d mapPose(double x, double y, double headingDegrees) {
        return new Pose2d(x * reflect, y * reflect, Math.toRadians(headingDegrees * reflect));
    }

    public static void setVoltage(double voltage) {
        Globals.voltage = voltage;
        Log.i("Globals", "set voltage to " + voltage);
    }

    public static double correctPower(double power) {
        return power * 13.0 / Globals.voltage;
    }

    public static void autoRunningTelemetry(Telemetry telemetry) {
        telemetry.addData("Runtime", Globals.runtime.seconds());
    }

    public static double time() {
        return runtime.milliseconds();
    }

    public static double timeSince(double time) {
        return runtime.milliseconds() - time;
    }
}
