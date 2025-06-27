package org.firstinspires.ftc.teamcode.blucru.common.path_base.sample;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeKinematics;

public class SampleIntakeAtPointPath extends PIDPathBuilder {
    public SampleIntakeAtPointPath(Pose2d rawDrivePoint, Pose2d rawBlockPose) {
        super();
        double x = rawDrivePoint.getX();
        double y = rawDrivePoint.getY();
        double h = rawDrivePoint.getHeading();
        Pose2d drivePointCopy = new Pose2d(x, y, h);

        Log.i("ROBOT POSE", "Drive Point: " + drivePointCopy.getX() + ", " + drivePointCopy.getY() + ", " + drivePointCopy.getHeading());
//        rawBlockPose = Globals.mapPose(rawBlockPose);

        double[] poseToInverseKinematics = BoxtubeKinematics.getExtensionTurretPose(rawBlockPose.vec().minus(drivePointCopy.vec()).rotated(-drivePointCopy.getHeading()));

        double spinWristAngle = rawBlockPose.getHeading() - rawDrivePoint.getHeading() - Math.PI/2;

//        Log.i("SampleIntakeAtPointPath", "Block pose:" + rawBlockPose);
//        Log.i("SampleIntakeAtPointPath", "Drive point:" + rawDrivePoint);
//        Log.i("SampleIntakeAtPointPath", "Block heading:" + rawBlockPose.getHeading());
//        Log.i("SampleIntakeAtPointPath", "Drive point to block heading:" + rawBlockPose.vec().minus(rawDrivePoint).angle());
//        Log.i("SampleIntakeAtPointPath", "Wrist final: " + rawWristFinal);
//        Log.i("SampleIntakeAtPointPath", "Point to point mag: " + rawDrivePoint.minus(rawBlockPose.vec()).norm());
//        Log.i("SampleIntakeAtPointPath", "x: " + x);

        Log.i("ROBOT POSE", "Drive Point: " + drivePointCopy.getX() + ", " + drivePointCopy.getY() + ", " + drivePointCopy.getHeading());
        this.setPower(0.7)
                .addPoint(rawDrivePoint, 10)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ArmCommand(0.4),
                            new UpDownWristAngleCommand(-Math.PI/2 + 0.05),
                            new WaitCommand(300),
                            new ClawOpenCommand(),
                            new ExtensionCommand(poseToInverseKinematics[0]),
                            new SpinWristGlobalAngleCommand(rawBlockPose.getHeading()),
//                            new SpinWristAngleCommand(spinWristAngle),
                            new WaitCommand(80),
                            new TurretAngleCommand(poseToInverseKinematics[1])
                    ).schedule();
                })

                .waitMillis(900)
                .addPoint(rawDrivePoint)
                .callback(() -> {
                    new GrabCommand().schedule();
                })
                .waitMillis(300);

    }
}
