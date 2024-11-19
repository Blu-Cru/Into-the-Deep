package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristHorizontalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenCycleIntakePath extends PIDPathBuilder {
    public SpecimenCycleIntakePath() {
        super();
        this.setPower(0.7)
                .addMappedPoint(18, -40, 270, 8)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(300),
                        new WristHorizontalCommand(),
                        new ArmGlobalAngleCommand(0),
                        new BoxtubeExtendCommand(0.27, 8)
                ))
                .setPower(0.4)
                .addMappedPoint(27, -48, -60)
                .schedule(new SequentialCommandGroup(
                        new WheelIntakeCommand(),
                        new ClampReleaseCommand()
                ))
                .waitMillis(600)
                .setPower(0.2)
                .addMappedPoint(29, -49.5, -60)
                .waitMillis(1300);
    }
}
