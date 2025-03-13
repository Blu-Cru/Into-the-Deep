package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;

public class SampleIntakeAtPointPath extends PIDPathBuilder {
    public SampleIntakeAtPointPath(Vector2d rawDrivePoint, Pose2d rawBlockPose) {
        super();
//
//        rawBlockPose = Globals.mapPose(rawBlockPose);

        double wristHeading = -Angle.normDelta(rawBlockPose.getHeading() - rawBlockPose.vec().minus(rawDrivePoint).angle());

        double rawWristFinal;
        if(wristHeading > 0.0) rawWristFinal = wristHeading - Math.PI;
        else rawWristFinal = wristHeading;

        double x = Range.clip(rawDrivePoint.minus(rawBlockPose.vec()).norm() + Math.sin(rawWristFinal) * BoxtubeKinematics.WRIST_y, 0, 26);

//        Log.i("SampleIntakeAtPointPath", "Block pose:" + rawBlockPose);
//        Log.i("SampleIntakeAtPointPath", "Drive point:" + rawDrivePoint);
//        Log.i("SampleIntakeAtPointPath", "Block heading:" + rawBlockPose.getHeading());
//        Log.i("SampleIntakeAtPointPath", "Drive point to block heading:" + rawBlockPose.vec().minus(rawDrivePoint).angle());
//        Log.i("SampleIntakeAtPointPath", "Wrist final: " + rawWristFinal);
//        Log.i("SampleIntakeAtPointPath", "Point to point mag: " + rawDrivePoint.minus(rawBlockPose.vec()).norm());
//        Log.i("SampleIntakeAtPointPath", "x: " + x);

        this.setPower(0.7)
                .addTurnToPoint(rawDrivePoint, rawBlockPose.vec(), 4)
                .callback(() -> {
                    new BoxtubeSplineCommand(
                            new Pose2d(x, 6, -Math.PI/2),
                            rawWristFinal,
                            0.6
                    ).schedule();
                })
                .waitMillis(700)
                .addTurnToPoint(rawDrivePoint, rawBlockPose.vec())
                .callback(() -> {
                    new WheelIntakeCommand().schedule();
                    new ClampReleaseCommand().schedule();
                    new BoxtubeSplineCommand(
                            new Pose2d(x, 2, -Math.PI/2),
                            rawWristFinal,
                            0.28
                    ).schedule();
                })
                .waitMillis(750);

    }
}
