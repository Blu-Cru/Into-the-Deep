package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.GetHooksHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.BoxtubeHooksTopBarCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.BoxtubeRetractHang3Command;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.servo.HangServosHangComamnd;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.servo.HangServosReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class HangFullTest extends BluLinearOpMode {
    enum State {
        RETRACT,
        RELEASED,
        UP
    }

    StateMachine sm;

    @Override
    public void initialize() {
        addPivot();
        addExtension();
        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());
        addArm();
        addClaw();
        addHangMotor();
        addHangServos();

        sm = new StateMachineBuilder()
                .state(State.RETRACT)
                .transition(() -> stickyG1.b, State.RELEASED, () -> {
                    new HangServosReleaseCommand().schedule();
                    new GetHooksHighCommand().schedule();
                })

                .state(State.RELEASED)
                .transition(() -> stickyG1.b, State.UP, () -> {
                    new SequentialCommandGroup(
                            new HangServosHangComamnd(),
                            new WaitCommand(200),
                            new BoxtubeHooksTopBarCommand()
                    ).schedule();
                })
                .transition(() -> stickyG1.a, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })

                .state(State.UP)
                .transition(() -> stickyG1.b, State.RETRACT, () -> {
                    new BoxtubeRetractHang3Command().schedule();
                })
                .build();

        sm.setState(State.RETRACT);
        sm.start();
    }

    @Override
    public void periodic() {
        if(stickyG1.dpad_down) hangServos.toggle();
        hangMotor.setManualPower(-gamepad1.left_stick_y);

        sm.update();
    }
}
