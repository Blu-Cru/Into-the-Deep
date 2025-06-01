package org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawLooseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.spinwrist.SpinWristCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.updownwrist.UpDownWristRetractCommand;

public class EndEffectorRetractCommand extends SequentialCommandGroup {
    public EndEffectorRetractCommand() {
        super(
                new ClawLooseCommand(),
                new ArmRetractCommand(),
                new SpinWristCenterCommand(),
                new TurretCenterCommand(),
                new UpDownWristRetractCommand()
        );
    }
}
