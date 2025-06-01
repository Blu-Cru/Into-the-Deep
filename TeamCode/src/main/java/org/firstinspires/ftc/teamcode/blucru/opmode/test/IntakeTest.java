package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.spinwrist.SpinWristCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class IntakeTest extends BluLinearOpMode {
    enum State{
        RETRACTED,
        PREINTAKE,
        GRABBED
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
                .transition(() -> stickyG1.left_bumper, State.PREINTAKE, () -> {
                    new SequentialCommandGroup(
                            new TurretCenterCommand(),
                            new SpinWristCenterCommand(),
                            new PreIntakeCommand()
                    ).schedule();
                })

                .state(State.PREINTAKE)
                .transition(() -> stickyG1.left_bumper, State.GRABBED, () -> {
                    new GrabCommand().schedule();
                })
                .transition(() -> stickyG1.a, State.RETRACTED, () -> {

                })
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
