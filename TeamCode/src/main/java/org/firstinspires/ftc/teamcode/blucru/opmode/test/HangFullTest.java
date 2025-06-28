package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.BoxtubeHooksTopBarCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.BoxtubeRetractFromTopBarCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.BoxtubeRetractHang3Command;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.GetHooksCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.HooksHighBarReadyCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.HooksOnHighBarCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.PTOHangCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.pto.PTOEngageCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.servo.slides.HangServosHangComamnd;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.servo.slides.HangServosReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class HangFullTest extends BluLinearOpMode {
    enum State {
        RETRACT,
        HANG_READY_HIGH_BAR,
        HOOKS_ON_HIGH_BAR,
        PULLING_UP
    }

    StateMachine sm;

    @Override
    public void initialize() {
        addPTODrivetrain();
        addHangServos();
        addPTOServos();
        addClapServos();
        addPivot();
        addExtension();
        addArm();
        addClaw();
        addUpDownWrist();
        addSpinWrist();
        addTurret();

        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());

        sm = new StateMachineBuilder()
                .state(State.RETRACT)
                .transition(() -> stickyG1.dpad_up, State.HANG_READY_HIGH_BAR, () -> {
                    slideHangServos.release();
                    new SequentialCommandGroup(
                            new HangServosReleaseCommand(),
                            new GetHooksCommand(),
                            new WaitCommand(400),
                            new HooksHighBarReadyCommand()
                    ).schedule();
                })
                .loop(() -> ptoDt.teleOpDrive(gamepad1))

                .state(State.HANG_READY_HIGH_BAR)
                .transition(() -> stickyG1.dpad_up, State.HOOKS_ON_HIGH_BAR, () -> {
                    slideHangServos.hang();
                    new SequentialCommandGroup(
                            new HangServosHangComamnd(),
                            new PTOEngageCommand(),
                            new WaitCommand(200),
                            new HooksOnHighBarCommand()
                    ).schedule();
                })
                .transition(() -> stickyG1.a, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })

                .state(State.HOOKS_ON_HIGH_BAR)
                .transition(() -> stickyG1.dpad_up, State.PULLING_UP, () -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(200),
                            new PTOHangCommand(),
                            new ArmRetractCommand(),
                            new WaitCommand(200),
                            new BoxtubeRetractFromTopBarCommand()
                    ).schedule();
                })
                .transition(() -> stickyG1.a, State.RETRACT, () -> {
                    new BoxtubeRetractFromTopBarCommand().schedule();
                })

                .state(State.PULLING_UP)
                .transition(() -> stickyG1.a, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })
                .build();

        sm.setState(State.RETRACT);
        sm.start();
    }

    @Override
    public void periodic() {
        sm.update();

        switch(Enum.valueOf(State.class, sm.getStateString())) {
            case RETRACT:
            case HANG_READY_HIGH_BAR:
                ptoDt.teleOpDrive(gamepad1);
                break;
        }

        if(stickyG1.right_stick_button) {
            ptoDt.setHeading(Math.PI/2);
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", sm.getState());
    }
}
