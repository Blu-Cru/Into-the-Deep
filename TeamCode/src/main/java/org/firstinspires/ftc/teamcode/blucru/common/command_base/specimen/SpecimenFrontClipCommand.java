package org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;

public class SpecimenFrontClipCommand extends SequentialCommandGroup {
    public SpecimenFrontClipCommand() {
        super(
                new ArmCommand(0.0),
                new WaitCommand(250),
                new UpDownWristAngleCommand(0.05),
                new SpinWristAngleCommand(Math.PI),
                new BoxtubeCommand(0.67, 11.0)
        );
    }
}
