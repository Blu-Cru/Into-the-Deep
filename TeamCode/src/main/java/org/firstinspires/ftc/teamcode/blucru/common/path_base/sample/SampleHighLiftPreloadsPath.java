package org.firstinspires.ftc.teamcode.blucru.common.path_base.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmBottomHardStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawLooseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.sample.SampleBackHighAutoCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.sample.SampleBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class SampleHighLiftPreloadsPath extends PIDPathBuilder {
    public SampleHighLiftPreloadsPath() {
        super();
        this.setPower(0.8)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ClawLooseCommand(),
                            new SpinWristCenterCommand(),
                            new TurretCenterCommand(),
                            new UpDownWristRetractCommand(),
                            new ArmBottomHardStopCommand(),
                            new BoxtubeCommand(Math.PI/2, 1.5),
                            new WaitCommand(50),
                            new SampleBackHighAutoCommand()
                    ).schedule();
                })
                .addMappedPoint(-58, -58, 45);
    }
}