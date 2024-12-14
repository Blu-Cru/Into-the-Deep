package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.spline.TimedEndHermiteSpline;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.pose.BoxtubeIKPoseVelocity;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Wrist;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;

public class BoxtubeSpline extends TimedEndHermiteSpline {
    public MotionProfile blockAngleProfile, wristAngleProfile;
    public Pose2d startPose, endPose;
    public Vector2d startTangent;
    double endWristAngle;
    public BoxtubeIKPoseVelocity states;

    public BoxtubeSpline(Pose2d startPose, Vector2d startTangent, Pose2d endPose, double endWristAngle, double durationSecs) {
        super(startPose.vec(), startTangent, endPose.vec(), durationSecs);
        this.startPose = startPose;
        this.endPose = endPose;
        this.startTangent = startTangent;
        this.endWristAngle = endWristAngle;

        states = new BoxtubeIKPoseVelocity(this);
    }

    public BoxtubeSpline start() {
        Log.i("BoxtubeSpline", "spline started");
        super.start();
        blockAngleProfile = getTimeConstrainedMotionProfile(startPose.getHeading(), endPose.getHeading(), duration).start();
        wristAngleProfile = getTimeConstrainedMotionProfile(Robot.getInstance().wrist.getAngle(), endWristAngle, duration).start();

        return this;
    }

    public void update() {
        states = new BoxtubeIKPoseVelocity(this);
    }

    public MotionProfile getTimeConstrainedMotionProfile(double start, double end, double time) {
        // profile that accelerates in the first 25% of the time and decelerates in the last 75% of the time
        double aMax = (Math.abs(start - end)) / (time * 0.25 * time * 0.75);
        double vMax = aMax * time * 0.25;

        return new MotionProfile(end, start, vMax, aMax).start();
    }

    public void telemetry() {
        Telemetry tele = Globals.tele;
        tele.addData("Block Pose", states.blockPose);
        tele.addData("Block Vel", states.blockVel);
        tele.addData("Pivot State", states.pivotState);
        tele.addData("Extension State", states.extensionState);
        tele.addData("Arm Angle", states.armAngle);
        tele.addData("Wrist Angle", states.wristAngle);
        tele.addData("started", started);
    }

    public void testTelemetry() {
        Telemetry tele = Globals.tele;
        tele.addData("Block Pose", states.blockPose);
        tele.addData("Block Vel", states.blockVel);
        tele.addData("Pivot Pos", states.pivotState.getX());
        tele.addData("Pivot Vel", states.pivotState.getY());
        tele.addData("Extension Pose", states.extensionState.getX());
        tele.addData("Extension Vel", states.extensionState.getY());
        tele.addData("Spline arm Angle", states.armAngle);
        tele.addData("Spline wrist Angle", states.wristAngle);
        tele.addData("started: ", started);
    }
}
