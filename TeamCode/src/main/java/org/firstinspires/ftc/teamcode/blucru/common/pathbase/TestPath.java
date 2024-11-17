package org.firstinspires.ftc.teamcode.blucru.common.pathbase;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.BasketBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.BasketBackLowCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmDropToGroundCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmPreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class TestPath extends PIDPathBuilder {
    public TestPath() {
        super();
        this.setPower(0.35)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(200),
                        new BasketBackHighCommand()
                ))
                .addMappedPoint(50, 50, 225)
                .setPower(0.2)
                .addMappedPoint(54, 54, 225)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(150),
                        new ClampReleaseCommand(),
                        new WheelReverseCommand()
                ))
                .waitMillis(400)
                .schedule(new SequentialCommandGroup(
                        new ArmGlobalAngleCommand(1.5),
                        new WaitCommand(250),
                        new BoxtubeRetractCommand(),
                        new EndEffectorRetractCommand()
                ))
                .setPower(0.2)
                .addMappedPoint(40, 50, 270, 6)
                .setPower(0.5)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(400),
                        new ExtensionCommand(1),
                        new WaitCommand(150),
                        new ArmDropToGroundCommand(),
                        new WheelIntakeCommand(),
                        new ClampReleaseCommand()
                ))
                .addMappedPoint(32, 31, -18, 3)
                .schedule(new SequentialCommandGroup(
                        new WaitCommand(400),
                        new ExtensionCommand(6)
                ))
                .waitMillis(1000);
    }
}
