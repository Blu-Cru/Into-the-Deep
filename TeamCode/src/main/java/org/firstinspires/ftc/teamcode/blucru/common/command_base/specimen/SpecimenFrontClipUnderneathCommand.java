package org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmBottomHardStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;

public class SpecimenFrontClipUnderneathCommand extends SequentialCommandGroup {
    /*
    Underneath: 9.5, 0.81
    Above: 12.63, 0.73

    underneath: 11.8, 0.72
    above: 12.5, 0.746
     */
    public SpecimenFrontClipUnderneathCommand(int waitMillis) {
        super(
                new ArmBottomHardStopCommand(),
                new WaitCommand(130),
                new PivotCommand(1.0),
                new ExtensionCommand(4.0),
                new WaitCommand(waitMillis),
                new UpDownWristAngleCommand(0.05),
                new SpinWristAngleCommand(Math.PI),
                new BoxtubeCommand(0.84, 10.5)
        );
    }

    public SpecimenFrontClipUnderneathCommand() {
        this(200);
    }
}
