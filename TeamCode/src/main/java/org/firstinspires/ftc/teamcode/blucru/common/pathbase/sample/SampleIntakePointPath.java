package org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeKinematics;

public class SampleIntakePointPath extends PIDPathBuilder {
    public SampleIntakePointPath(Vector2d drivePoint, Pose2d intakePoint) {
        super();

        double distanceToPoint = drivePoint.minus(intakePoint.vec()).norm();
        double heading = drivePoint.minus(intakePoint.vec()).angle();
        double wristAngle = intakePoint.getHeading() - heading;

        this.setPower(0.7)
                .addMappedTurnToPoint(drivePoint, intakePoint.vec(), 7)
                .callback(() -> {
                    new BoxtubeSplineCommand(
                            new Pose2d(distanceToPoint - Math.sin(wristAngle) * BoxtubeKinematics.WRIST_y, 6, -Math.PI/2),
                            wristAngle,
                            0.5
                    ).schedule();
                })
                .waitMillis(450)
                .addMappedTurnToPoint(drivePoint, intakePoint.vec())
                .callback(() -> {
                    new WheelIntakeCommand().schedule();
                    new ClampReleaseCommand().schedule();
                    new BoxtubeSplineCommand(
                            new Pose2d(distanceToPoint - Math.sin(wristAngle) * BoxtubeKinematics.WRIST_y, 2, -Math.PI/2),
                            wristAngle,
                            0.25
                    ).schedule();
                })
                .waitMillis(1000);
    }
}
