package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawLooseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.SpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class IntakeTest extends BluLinearOpMode {
    enum State{
        RETRACTED,
        PREINTAKE,
        INTAKING_SPEC,
        SCORING_SPEC,
        SCORING_BASKET,

        GRABBED_GROUND,
        SENSING_GROUND,
        GRABBED_SPEC,
        SENSING_SPEC,
    }

    StateMachine sm;
    double spinWristGlobalAngle = 0;

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
        addPusher();
        addPTOServos();

        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());

        dt.setDrivePower(0.7);

        sm = new StateMachineBuilder()
                .state(State.RETRACTED)
                .onEnter(() -> dt.setDrivePower(1.0))
                .transition(() -> (stickyG1.left_bumper || stickyG2.left_bumper) && pivot.getAngle() < 0.2, State.PREINTAKE, () -> {
                    new SequentialCommandGroup(
                            new TurretCenterCommand(),
                            new PreIntakeCommand(),
                            new SpinWristGlobalAngleCommand(0),
                            new WaitCommand(180),
                            new ExtensionCommand(15)
                    ).schedule();
                    spinWristGlobalAngle = 0;
                })
                .transition(() -> stickyG1.y && extension.getDistance() < 2.0, State.SCORING_BASKET, () -> {
                    new SequentialCommandGroup(
                            new ArmCommand(0.2),
                            new BoxtubeCommand(Math.PI/2, 24.0),
                            new UpDownWristAngleCommand(1.5),
                            new SpinWristAngleCommand(Math.PI),
                            new TurretCenterCommand()
                    ).schedule();
                })
                .transition(() -> stickyG2.x, State.INTAKING_SPEC, () -> {
                    new SpecimenIntakeCommand().schedule();
                })
                .loop(() -> {
                    if(stickyG1.right_bumper || stickyG2.right_bumper) new SpitCommand().schedule();
                })

                .state(State.PREINTAKE)
                .onEnter(() -> {
                    spinWrist.setTurretGlobalAngle(spinWristGlobalAngle);
                    dt.setDrivePower(0.45);
                })
                .transition(() -> stickyG1.a || stickyG2.a, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> stickyG1.left_bumper || stickyG2.left_bumper, State.GRABBED_GROUND, () -> {
                    new GrabCommand().schedule();
                })

                .loop(() -> {
                    double rightInput = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
                    double leftInput = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
                    if(rightInput > 0.1) {
                        turret.setAngle(-rightInput);
                    } else if (leftInput > 0.1){
                        turret.setAngle(leftInput);
                    } else {
                        turret.center();
                    }

                    if(stickyG1.dpad_up || stickyG2.dpad_up) {
                        setSpinWristGlobalAngle(0);
                    } else if (stickyG1.dpad_right || stickyG2.dpad_right) {
                        setSpinWristGlobalAngle(Math.PI/2);
                    }
                })

                .state(State.GRABBED_GROUND)
                .transitionTimed(0.63, State.SENSING_GROUND)
                .state(State.SENSING_GROUND)
                .transition(() -> cactus.validSample, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> cactus.isEmpty(), State.PREINTAKE, () -> {
                    new PreIntakeCommand().schedule();
                })
                .transitionTimed(0.15, State.PREINTAKE, () -> {
                    new PreIntakeCommand().schedule();
                })

                .state(State.SCORING_BASKET)
                .onEnter(() -> {
                    dt.setDrivePower(0.55);
                })
                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new ArmCommand(0),
                            new WaitCommand(50),
                            new UpDownWristAngleCommand(-1),
                            new WaitCommand(100),
                            new UpDownWristAngleCommand(-1),
                            new WaitCommand(150),
                            new ArmRetractCommand(),
                            new BoxtubeRetractCommand(),
                            new ClawLooseCommand(),
                            new SpinWristCenterCommand(),
                            new TurretCenterCommand(),
                            new UpDownWristRetractCommand()
                    ).schedule();
                })

                .state(State.INTAKING_SPEC)
                .onEnter(() -> dt.setDrivePower(0.6))
                .transition(() -> stickyG2.a, State.RETRACTED, () -> new FullRetractCommand().schedule())
                .transition(() -> (stickyG2.left_bumper || cactus.validSample) && pivot.getAngle() > 1.3, State.GRABBED_SPEC, () -> {
                    new SequentialCommandGroup(
                            new ClawGrabCommand(),
                            new WaitCommand(120),
                            new PivotCommand(1.0),
                            new UpDownWristAngleCommand(-2.0)
                    ).schedule();
                })

                .state(State.GRABBED_SPEC)
                .transitionTimed(0.3, State.SENSING_SPEC)
                .state(State.SENSING_SPEC)
                .transition(() -> cactus.isEmpty(), State.INTAKING_SPEC, () -> {
                    new SpecimenIntakeCommand().schedule();
                })
                .transitionTimed(0.1, State.INTAKING_SPEC, () -> {
                    new SpecimenIntakeCommand().schedule();
                })
                .transition(() -> cactus.validSample, State.SCORING_SPEC, () -> {
                    new SequentialCommandGroup(
                            new PivotCommand(0.63),
                            new ExtensionCommand(4.0),
                            new ArmCommand(0),
                            new WaitCommand(150),
                            new UpDownWristAngleCommand(0.5),
                            new SpinWristCenterCommand(),
                            new WaitCommand(200),
                            new ExtensionCommand(10.0),
                            new WaitCommand(200),
                            new ClawLooseCommand(),
                            new WaitCommand(150),
                            new ClawGrabCommand()
                    ).schedule();
                })

                .state(State.SCORING_SPEC)
                .onEnter(() -> dt.setDrivePower(0.8))
                .transition(() -> stickyG2.left_bumper, State.INTAKING_SPEC, () -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(170),
                            new ExtensionRetractCommand(),
                            new ArmCommand(2.8),
                            new WaitCommand(100),
                            new SpecimenIntakeCommand()
                    ).schedule();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(200),
                            new FullRetractCommand()
                    ).schedule();
                })

                .build();

        sm.setState(State.RETRACTED);
        sm.start();
    }

    @Override
    public void periodic() {
        if (gamepad1.getGamepadId() == -1) dt.teleOpDrive(gamepad2);
        else dt.teleOpDrive(gamepad1);

        if(gamepad1.right_stick_button || gamepad2.right_stick_button) {
            dt.setHeading(Math.PI/2);
            gamepad1.rumble(150);
        }
        sm.update();
    }

    public void setSpinWristGlobalAngle(double angle) {
        spinWristGlobalAngle = angle;
        spinWrist.setTurretGlobalAngle(angle);
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", sm.getState());
        telemetry.addData("Spin wrist angle", spinWristGlobalAngle);
    }
}
