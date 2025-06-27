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

import java.util.Arrays;

public class SampleIntakeAtPointPath extends PIDPathBuilder {
    public SampleIntakeAtPointPath(Pose2d rawDrivePoint, Pose2d rawBlockPose) {
        super();
        Log.i("ROBOT POSE", "Drive Point: " + rawDrivePoint.getX() + ", " + rawDrivePoint.getY() + ", " + rawDrivePoint.getHeading());
//        rawBlockPose = Globals.mapPose(rawBlockPose);

        double[] poseToInverseKinematics = BoxtubeKinematics.getExtensionTurretPose(rawBlockPose.vec().minus(rawDrivePoint.vec()).rotated(-rawDrivePoint.getHeading()));

//        double spinWristAngle = rawBlockPose.getHeading() - rawDrivePoint.getHeading() - Math.PI/2;

        Log.i("SampleIntakeAtPointPath", "Block pose:" + rawBlockPose);
        Log.i("SampleIntakeAtPointPath", "Drive point:" + rawDrivePoint);
        Log.i("SampleIntakeAtPointPath", "Joint poses:" + Arrays.toString(poseToInverseKinematics));
//        Log.i("SampleIntakeAtPointPath", "Wrist final: " + spinWristAngle);

        this.setPower(0.7)
                .addPoint(rawDrivePoint, 10)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ArmCommand(0.3),
                            new UpDownWristAngleCommand(-Math.PI/2 + 0.05),
                            new WaitCommand(300),
                            new ClawOpenCommand(),
                            new ExtensionCommand(poseToInverseKinematics[0]),
                            new SpinWristGlobalAngleCommand(-rawBlockPose.getHeading()),
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
