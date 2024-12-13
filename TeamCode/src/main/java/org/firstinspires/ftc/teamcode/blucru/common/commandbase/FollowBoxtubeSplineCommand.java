package org.firstinspires.ftc.teamcode.blucru.common.commandbase;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeSpline;

public class FollowBoxtubeSplineCommand extends InstantCommand {
    public FollowBoxtubeSplineCommand(Vector2d startVelocity, Pose2d endPose, double wristAngle, double duration) {
        super(() -> {
            BoxtubeSpline spline = new BoxtubeSpline(Robot.getInstance().getBoxtubePose(), startVelocity, endPose, wristAngle, duration);
            Robot.getInstance().followBoxtubeSpline(spline);
        });

        addRequirements(Robot.getInstance().dt,
                Robot.getInstance().wrist,
                Robot.getInstance().pivot,
                Robot.getInstance().extension,
                Robot.getInstance().arm);
    }
}
