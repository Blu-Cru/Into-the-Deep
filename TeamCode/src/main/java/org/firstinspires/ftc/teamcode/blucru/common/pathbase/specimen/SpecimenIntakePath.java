package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristOppositeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenIntakePath extends PIDPathBuilder {
    public SpecimenIntakePath(boolean spline) {
        super();
        if(spline) {
            this.setPower(0.7)
                    .addMappedPoint(24, -48.5, -60, 5)
                    .schedule(new SequentialCommandGroup(
                            new BoxtubeSplineCommand(
                                    new Pose2d(21, 10.1, 0.1),
                                    -Math.PI,
                                    0.5),
                            new WheelIntakeCommand(),
                            new ClampReleaseCommand()
                    ))
                    .waitMillis(400)
                    .setPower(0.25)
                    .addMappedPoint(29, -53, -60)
                    .waitMillis(350);
        } else {
            this.setPower(0.7)
                    .addMappedPoint(27, -48, -60, 5)
                    .schedule(new SequentialCommandGroup(
                            new WristOppositeCommand(),
                            new ArmGlobalAngleCommand(0),
                            new BoxtubeCommand(0.27, 8),
                            new WaitCommand(300),
                            new WheelIntakeCommand(),
                            new ClampReleaseCommand()
                    ))
                    .addMappedPoint(27, -48.5, -60)
                    .waitMillis(200)
                    .setPower(0.2)
                    .addMappedPoint(29, -53, -60)
                    .waitMillis(500);
        }
    }

    public SpecimenIntakePath() {
        this(true);
    }
}
