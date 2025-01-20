package org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube;

import android.util.Log;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;

public class BoxtubeRetractCommand extends SequentialCommandGroup {
    public BoxtubeRetractCommand() {
        super(
                new SequentialCommandGroup(
                        new ExtensionRetractCommand(),

                        // Wait time depends on distance extension needs to extend. Longer extension requires longer wait time.
                        // here i do distance in inches times 25, 25 inches will wait
                        new WaitCommand((long) (Robot.getInstance().extension.getDistance() * 12.0)),
                        new PivotRetractCommand()
                )
        );

        Log.i("BoxtubeRetractCommand", "Extension distance: " + Robot.getInstance().extension.getDistance());
    }

    public BoxtubeRetractCommand(long waitMillis) {
        super(
                new SequentialCommandGroup(
                        new ExtensionRetractCommand(),

                        // Wait time depends on distance extension needs to extend. Longer extension requires longer wait time.
                        // here i do distance in inches times 25, 25 inches will wait
                        new WaitCommand(waitMillis),
                        new PivotRetractCommand()
                )
        );

        Log.i("BoxtubeRetractCommand", "Wait time" + waitMillis);
    }
}
