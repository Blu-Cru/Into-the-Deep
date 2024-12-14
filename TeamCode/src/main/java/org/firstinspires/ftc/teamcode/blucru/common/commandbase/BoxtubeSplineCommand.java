package org.firstinspires.ftc.teamcode.blucru.common.commandbase;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeSpline;

public class BoxtubeSplineCommand extends InstantCommand {
    public BoxtubeSplineCommand(Vector2d startVelocity, Pose2d endPose, double wristAngle, double duration) {
        super(() -> {
            BoxtubeSpline spline = new BoxtubeSpline(Robot.getInstance().getBoxtubePose(), startVelocity, endPose, wristAngle, duration);
            Robot.getInstance().followBoxtubeSpline(spline);
        });

        addRequirements(
                Robot.getInstance().wrist,
                Robot.getInstance().pivot,
                Robot.getInstance().extension,
                Robot.getInstance().arm);
    }

    public BoxtubeSplineCommand(Pose2d endPose, double wristAngle, double duration) {
        this(new Vector2d(0,0),
                endPose,
                wristAngle,
                duration);
    }
}
