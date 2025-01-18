package org.firstinspires.ftc.teamcode.blucru.common.commandbase.spline;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeSpline;

public class BoxtubeSplineCommand extends InstantCommand {
    public BoxtubeSplineCommand(Vector2d startVelocity, Pose2d endPose, double wristAngle, double duration) {
        this(startVelocity, endPose, new Vector2d(0,0), wristAngle, duration);

        addRequirements(
                Robot.getInstance().wrist,
                Robot.getInstance().pivot,
                Robot.getInstance().extension,
                Robot.getInstance().arm);
    }

    public BoxtubeSplineCommand(Pose2d endPose, double wristAngle, double duration) {
        this(new Vector2d(0,0),
                endPose,
                new Vector2d(0,0),
                wristAngle,
                duration);
    }

    public BoxtubeSplineCommand(Vector2d startVelocity, Pose2d endPose, Vector2d endVelocity, double wristAngle, double duration) {
        super(() -> {
            BoxtubeSpline spline = new BoxtubeSpline(Robot.getInstance().getBoxtubePose(),
                    startVelocity,
                    endPose, endVelocity,
                    wristAngle,
                    duration);
            Robot.getInstance().followBoxtubeSpline(spline);

            Log.i("BoxtubeSplineCommand", "created spline. START POSE: " + Robot.getInstance().getBoxtubePose() +
                    "v0: " + startVelocity +
                    ", endPose: " + endPose +
                    ", v1: " + endVelocity +
                    ", wristAngle: " + wristAngle +
                    ", duration: " + duration);
        });

        addRequirements(
                Robot.getInstance().wrist,
                Robot.getInstance().pivot,
                Robot.getInstance().extension,
                Robot.getInstance().arm);
    }

    public BoxtubeSplineCommand(Pose2d startPose, Vector2d startVelocity, Pose2d endPose, Vector2d endVelocity, double wristAngle, double duration) {
        super(() -> {
            BoxtubeSpline spline = new BoxtubeSpline(startPose, startVelocity, endPose, endVelocity, wristAngle, duration);
            Robot.getInstance().followBoxtubeSpline(spline);

            Log.i("BoxtubeSplineCommand", "created spline. START POSE: " + Robot.getInstance().getBoxtubePose() +
                    "v0: " + startVelocity +
                    ", endPose: " + endPose +
                    ", v1: " + endVelocity +
                    ", wristAngle: " + wristAngle +
                    ", duration: " + duration);
        });

        addRequirements(
                Robot.getInstance().wrist,
                Robot.getInstance().pivot,
                Robot.getInstance().extension,
                Robot.getInstance().arm);
    }
}
