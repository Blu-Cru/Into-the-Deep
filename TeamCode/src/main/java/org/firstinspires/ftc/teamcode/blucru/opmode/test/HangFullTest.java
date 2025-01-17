package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.BoxtubeGetHooksCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.spline.BoxtubeSplineCommand;
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
        addWrist();
        addArm();
        addClamp();
        addWheel();
        addHangMotor();
        addHangServos();

        sm = new StateMachineBuilder()
                .state(State.RETRACT)
                .transition(() -> stickyG1.b, State.RELEASED, () -> {
                    hangServos.release();
                    new BoxtubeGetHooksCommand().schedule();
                })

                .state(State.RELEASED)
                .transition(() -> stickyG1.b, State.UP, () -> {
                    hangServos.retract();
                    new SequentialCommandGroup(
                            new WaitCommand(200),
                            new BoxtubeSplineCommand(
                                new Pose2d(7, 36, 1.5),
                                -Math.PI/2,
                                0.7
                            )
                    ).schedule();
                })
                .transition(() -> stickyG1.a, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })

                .state(State.UP)
                .transition(() -> stickyG1.b, State.RETRACT, () -> {
                    new SequentialCommandGroup(
                            new ExtensionRetractCommand(),
                            new ArmRetractCommand(),
                            new WaitCommand(600),
                            new PivotCommand(0.5)
                    ).schedule();
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
