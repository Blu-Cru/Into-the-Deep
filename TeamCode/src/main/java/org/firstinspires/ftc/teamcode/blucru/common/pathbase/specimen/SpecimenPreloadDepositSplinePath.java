package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontDunkRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenPreloadDepositSplinePath extends PIDPathBuilder {
    public SpecimenPreloadDepositSplinePath() {
        super();
        this.setPower(0.8)
                .schedule(new BoxtubeSplineCommand(
                        new Pose2d(6.4, 22.5, -0.3),
                        -Math.PI,
                        0.45
                ))
//                .schedule(new SpecimenFrontCommand())
                .addMappedPoint(7, -40, 90, 6)
                .setPower(0.45)
                .addMappedPoint(7, -33, 90)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(350),
                        new BoxtubeSplineCommand(
                                new Pose2d(6.4, 18, -0.3),
                                -Math.PI,
                                0.2
                        ),
                        new WaitCommand(300),
                        new ClampReleaseCommand(),
                        new WheelReverseCommand(),
//                        new SpecimenFrontDunkCommand(),
                        new SpecimenFrontDunkRetractCommand()
                ))
                .waitMillis(600);
    }
}
