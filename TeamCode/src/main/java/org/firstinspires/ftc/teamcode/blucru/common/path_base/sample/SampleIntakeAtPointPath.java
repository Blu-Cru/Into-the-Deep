package org.firstinspires.ftc.teamcode.blucru.common.path_base.sample;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeKinematics;

public class SampleIntakeAtPointPath extends PIDPathBuilder {
    public SampleIntakeAtPointPath(Pose2d drivePoint, Pose2d rawBlockPose) {
        super();
//
//        rawBlockPose = Globals.mapPose(rawBlockPose);

        double[] poseToInverseKinematics = BoxtubeKinematics.getExtensionTurretPose(rawBlockPose.vec().minus(drivePoint.vec()).rotated(-drivePoint.getHeading()));

        double spinWristAngle = rawBlockPose.getHeading() - drivePoint.getHeading();

//        Log.i("SampleIntakeAtPointPath", "Block pose:" + rawBlockPose);
//        Log.i("SampleIntakeAtPointPath", "Drive point:" + rawDrivePoint);
//        Log.i("SampleIntakeAtPointPath", "Block heading:" + rawBlockPose.getHeading());
//        Log.i("SampleIntakeAtPointPath", "Drive point to block heading:" + rawBlockPose.vec().minus(rawDrivePoint).angle());
//        Log.i("SampleIntakeAtPointPath", "Wrist final: " + rawWristFinal);
//        Log.i("SampleIntakeAtPointPath", "Point to point mag: " + rawDrivePoint.minus(rawBlockPose.vec()).norm());
//        Log.i("SampleIntakeAtPointPath", "x: " + x);

        this.setPower(0.7)
                .addMappedPoint(drivePoint.getX(), drivePoint.getY(), drivePoint.getHeading(), 10)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new PreIntakeCommand(),
                            new WaitCommand(170),
                            new ExtensionCommand(poseToInverseKinematics[0]),
                            new SpinWristAngleCommand(spinWristAngle),
                            new WaitCommand(80),
                            new TurretAngleCommand(poseToInverseKinematics[1])
                    ).schedule();
                })

                .waitMillis(600)
                .addMappedPoint(drivePoint.getX(), drivePoint.getY(), drivePoint.getHeading())
                .callback(() -> {
                    new GrabCommand().schedule();
                })
                .waitMillis(300);

    }
}
