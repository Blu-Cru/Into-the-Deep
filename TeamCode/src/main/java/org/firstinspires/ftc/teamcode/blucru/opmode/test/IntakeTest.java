package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.spinwrist.SpinWristCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.spinwrist.SpinWristGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.intake.SpitCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class IntakeTest extends BluLinearOpMode {
    enum State{
        RETRACTED,
        PREINTAKE,
        GRABBED,
        SENSING
    }

    StateMachine sm;

    @Override
    public void initialize() {
        addDrivetrain();
        addArm();
        addTurret();
        addUpDownWrist();
        addSpinWrist();
        addClaw();
        addPivot();
        addExtension();
        addCactus();

        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());

        dt.setDrivePower(0.7);

        sm = new StateMachineBuilder()
                .state(State.RETRACTED)
                .onEnter(() -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> stickyG1.left_bumper, State.PREINTAKE, () -> {
                    new SequentialCommandGroup(
                            new BoxtubeCommand(0, 10.0),
                            new TurretCenterCommand(),
                            new SpinWristCenterCommand(),
                            new PreIntakeCommand()
                    ).schedule();
                })
                .loop(() -> {
                    if(stickyG1.right_bumper) new SpitCommand().schedule();
                })

                .state(State.PREINTAKE)
                .onEnter(() -> new PreIntakeCommand().schedule())
                .transition(() -> stickyG1.a, State.RETRACTED)
                .transition(() -> stickyG1.left_bumper, State.GRABBED, () -> {
                    new GrabCommand().schedule();
                })

                .loop(() -> {
                    if(!stickyG1.right_trigger && !stickyG1.left_trigger) {
                        turret.center();
                    } else if(gamepad1.right_trigger > 0.1) {
                        turret.setAngle(-gamepad1.right_trigger);
                    } else {
                        turret.setAngle(gamepad1.left_trigger);
                    }

                    if(stickyG1.dpad_up) {
                        new SpinWristGlobalAngleCommand(0).schedule();
                    } else if (stickyG1.dpad_right) {
                        new SpinWristGlobalAngleCommand(-Math.PI/2).schedule();
                    }
                })

                .state(State.GRABBED)
                .transitionTimed(0.6, State.SENSING)

                .state(State.SENSING)
                .transition(() -> cactus.isEmpty(), State.PREINTAKE)
                .transition(() -> cactus.validSample, State.RETRACTED)
                .transitionTimed(0.15, State.PREINTAKE)
                .build();

        sm.setState(State.RETRACTED);
        sm.start();
    }

    @Override
    public void periodic() {
        dt.teleOpDrive(gamepad1);
        if(gamepad1.right_stick_button) {
            dt.setHeading(Math.PI/2);
            gamepad1.rumble(150);
        }
        sm.update();
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", sm.getState());
    }
}
