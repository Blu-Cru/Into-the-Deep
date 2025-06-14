package org.firstinspires.ftc.teamcode.blucru.common.command_base.hang;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;

public class BoxtubeRetractHang3Command extends SequentialCommandGroup {
    public BoxtubeRetractHang3Command() {
        super(
                new FullRetractCommand()
        );
    }
}
