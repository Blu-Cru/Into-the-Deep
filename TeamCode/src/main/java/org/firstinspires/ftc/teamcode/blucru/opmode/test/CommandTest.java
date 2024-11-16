package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Command Test", group = "test")
public class CommandTest extends BluLinearOpMode {
    @Override
    public void initialize() {
        addPivot();
        addExtension();
        addWrist();
        addWheel();
        addArm();
        addClamp();

        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());
    }

    @Override
    public void periodic() {
        if(stickyG1.a) {
            new BoxtubeRetractCommand().schedule();
        }

        if(stickyG1.dpad_up && !gamepad1.x) {
            new BoxtubeExtendCommand(1.6, 10.0).schedule();
        }

        if(stickyG1.dpad_left && !gamepad1.x) {
            new BoxtubeExtendCommand(1.6, 0).schedule();
        }

        if(stickyG1.dpad_right && !gamepad1.x) {
            new BoxtubeExtendCommand(1.6, 18).schedule();
        }

        if(stickyG1.dpad_up && !gamepad1.x) {
            new BoxtubeExtendCommand(1.6, 10.0).schedule();
        }
    }
}
