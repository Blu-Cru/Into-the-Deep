package org.firstinspires.ftc.teamcode.blucru.common.spline;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.Range;

public class TimedHermiteSpline extends CubicHermiteSpline {
    public double duration;
    double startTime;
    public boolean started;

    public TimedHermiteSpline(Vector2d startPose, Vector2d startVelocity, Vector2d endPose, Vector2d endVelocity, double durationSecs) {
        super(startPose, startVelocity, endPose, endVelocity);
        this.duration = durationSecs;
        startTime = Double.POSITIVE_INFINITY;
        started = false;
    }

    public TimedHermiteSpline(Vector2d startPose, Vector2d startVelocity, Vector2d endPose, double durationSecs) {
        super(startPose, startVelocity, endPose, new Vector2d(0,0));
        this.duration = durationSecs;
        startTime = Double.POSITIVE_INFINITY;
        started = false;
    }

    public TimedHermiteSpline(Vector2d startPose, Vector2d endPose, double durationSecs) {
        super(startPose, new Vector2d(0, 0), endPose, new Vector2d(0, 0));
        this.duration = durationSecs;
        started = false;
    }

    public TimedHermiteSpline start() {
        Log.i("TimedEndHermiteSpline", "spline started, time = " + System.currentTimeMillis());
        startTime = System.currentTimeMillis();
        started = true;
        return this;
    }

    public Vector2d getPosition() {
        double t = Range.clip((System.currentTimeMillis() - startTime) / duration / 1000.0, 0, 1);
        Log.d("TimedEndHermiteSpline", "getting position, t=" + t);
        return getPoint(t);
    }

    public Vector2d getVelocity() {
        double t = Range.clip((System.currentTimeMillis() - startTime) / duration / 1000.0, 0, 1);
        return getVelocity(t);
    }

    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= duration * 1000;
    }
}
