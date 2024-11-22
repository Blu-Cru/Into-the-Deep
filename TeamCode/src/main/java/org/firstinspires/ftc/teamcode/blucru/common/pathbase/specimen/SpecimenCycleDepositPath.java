package org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SpecimenCycleDepositPath extends PIDPathBuilder {
    public SpecimenCycleDepositPath(int scoreCount) {
        super();
        this.setPower(0.7)
                .schedule(new SequentialCommandGroup(
                        new WheelStopCommand(),
                        new ClampGrabCommand(),
                        new PivotCommand(1.2),
                        new WaitCommand(200),
                        new ExtensionCommand(2),
                        new ArmGlobalAngleCommand(2)
                ))
                .addMappedPoint(10 - scoreCount, -43, 270, 5)
                .schedule(
                        new SpecimenBackCommand()
                )
                .setPower(0.25)
                .addMappedPoint(7 - scoreCount * 1.5, -34, 270)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(350),
                        new SpecimenBackDunkCommand(),
                        new WaitCommand(100),
                        new WheelReverseCommand(),
                        new ClampReleaseCommand(),
                        new WaitCommand(150),
                        new PivotRetractCommand(),
                        new ExtensionRetractCommand(),
                        new ArmRetractCommand(),
                        new WristUprightForwardCommand()
                ))
                .waitMillis(450);
    }
}