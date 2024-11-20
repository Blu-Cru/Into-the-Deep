package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmDisableCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmEnableCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Park Position Test", group = "test")
public class ParkPositionTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addArm();
        addPivot();
    }

    @Override
    public void periodic() {
        if(stickyG1.b) {
            new SequentialCommandGroup(
                    new ArmGlobalAngleCommand(Math.PI/2),
                    new PivotCommand(Math.PI/2),
                    new WaitCommand(500),
                    new ArmDisableCommand()
            ).schedule();
        }

        if(stickyG1.a) {
            new SequentialCommandGroup(
                    new ArmEnableCommand(),
                    new ArmRetractCommand(),
                    new PivotRetractCommand()
            ).schedule();
        }
    }
}
