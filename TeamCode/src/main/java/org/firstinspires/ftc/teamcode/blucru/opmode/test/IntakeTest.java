package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.spinwrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.spinwrist.SpinWristCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.spinwrist.SpinWristGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.turret.TurretBackwardsCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.updownwrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.updownwrist.UpDownWristRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.intake.SpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.endeffector.Turret;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class IntakeTest extends BluLinearOpMode {
    enum State{
        RETRACTED,
        PREINTAKE,
        GRABBED,
        SENSING,
        INTAKING_SPEC,
        SCORING_BASKET
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
                .transition(() -> stickyG1.left_bumper && pivot.getAngle() < 0.2, State.PREINTAKE, () -> {
                    new SequentialCommandGroup(
                            new ExtensionCommand(8),
                            new TurretCenterCommand(),
                            new SpinWristCenterCommand(),
                            new PreIntakeCommand()
                    ).schedule();
                })
                .transition(() -> stickyG1.y && extension.getDistance() < 2.0, State.SCORING_BASKET, () -> {
                    new SequentialCommandGroup(
                            new ArmMotionProfileCommand(0.2),
                            new BoxtubeCommand(Math.PI/2, 24.0),
                            new UpDownWristAngleCommand(1.5),
                            new SpinWristAngleCommand(Math.PI),
                            new TurretCenterCommand()
                    ).schedule();
//                    new SequentialCommandGroup(
//                            new ClawGrabCommand(),
//                            new SpinWristCenterCommand(),
//                            new ArmMotionProfileCommand(Math.PI/2),
//                            new BoxtubeCommand(Math.PI/2, 10.0),
//                            new WaitCommand(500),
//                            new TurretBackwardsCommand(),
//                            new WaitCommand(500),
//                            new ArmMotionProfileCommand(2.8),
//                            new UpDownWristAngleCommand(-2.6)
//                    ).schedule();
                })
                .loop(() -> {
                    if(stickyG1.right_bumper) new SpitCommand().schedule();
                })

                .state(State.PREINTAKE)
                .onEnter(() -> new PreIntakeCommand().schedule())
                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> stickyG1.left_bumper, State.GRABBED, () -> {
                    new GrabCommand().schedule();
                })

                .loop(() -> {
                    if(gamepad1.right_trigger > 0.1) {
                        turret.setAngle(-gamepad1.right_trigger);
                    } else if (gamepad1.left_trigger > 0.1){
                        turret.setAngle(gamepad1.left_trigger);
                    } else {
                        turret.center();
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
                .transition(() -> cactus.validSample, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .transitionTimed(0.15, State.PREINTAKE)

                .state(State.SCORING_BASKET)
                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(150),
                            new UpDownWristAngleCommand(-1),
                            new ArmMotionProfileCommand(0),
                            new WaitCommand(200),
                            new ArmRetractCommand(),
                            new BoxtubeRetractCommand(),
                            new FullRetractCommand()
                    ).schedule();
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
