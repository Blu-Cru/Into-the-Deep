package org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.spline.TimedEndHermiteSpline;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.pose.BoxtubeIKPoseVelocity;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Wrist;
import org.firstinspires.ftc.teamcode.blucru.common.util.MotionProfile;

public class BoxtubeSpline extends TimedEndHermiteSpline {
    public MotionProfile blockAngleProfile, wristAngleProfile;
    Pose2d startPose, endPose;
    Vector2d startTangent;
    double duration, endWristAngle;
    public BoxtubeIKPoseVelocity states;

    public BoxtubeSpline(Pose2d startPose, Vector2d startTangent, Pose2d endPose, double endWristAngle, double durationSecs) {
        super(startPose.vec(), startTangent, endPose.vec(), durationSecs);
        this.startPose = startPose;
        this.endPose = endPose;
        this.startTangent = startTangent;
        this.endWristAngle = endWristAngle;
        this.duration = durationSecs;
    }

    public BoxtubeSpline start() {
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

        return new MotionProfile(start, end, vMax, aMax).start();
    }
}
