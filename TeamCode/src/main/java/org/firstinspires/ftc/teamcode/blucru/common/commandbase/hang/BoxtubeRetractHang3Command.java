package org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;

public class BoxtubeRetractHang3Command extends SequentialCommandGroup {
    public BoxtubeRetractHang3Command() {
        super(
                new FullRetractCommand()
        );
    }
}
