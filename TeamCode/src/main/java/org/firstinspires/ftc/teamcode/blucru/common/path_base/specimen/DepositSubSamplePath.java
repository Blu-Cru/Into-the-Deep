package org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;

public class DepositSubSamplePath extends PIDPathBuilder {

    public DepositSubSamplePath(){
        super();
        this.setPower(0.8)
                .callback(() -> {
                    new FullRetractCommand().schedule();
                })
                .waitMillis(2000) //value rn is for safety, tune it
                .addMappedPoint(20, -50, 90)
                .callback(() -> {
                    new SequentialCommandGroup(
                            new ExtensionCommand(10),
                            new ArmCommand(0),
                            new TurretMotionProfileCommand(-0.6),
                            new WaitCommand(400)
                    ).schedule();
                })
                .waitMillis(2000)
                .callback(() -> new ClawOpenCommand().schedule());

    }
}
