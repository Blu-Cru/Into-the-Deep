package org.firstinspires.ftc.teamcode.blucru.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.RetractFromBasketCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.SpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.sample.SampleBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenFrontClipUnderneathCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenFrontFlatCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackClipCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackFlatCommand;
import org.firstinspires.ftc.teamcode.blucru.common.util.SampleOrientation;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "2")
public class Solo extends BluLinearOpMode {
    enum State{
        HOME,
        PREINTAKE,
        INTAKING_SPEC,
        SCORING_SPEC,
        SCORING_BASKET,

        GRABBED_GROUND,
        SENSING_GROUND,
        GRABBED_SPEC,
        SENSING_SPEC,

        MANUAL_RESET
    }

    StateMachine sm;
    SampleOrientation orientation = SampleOrientation.VERTICAL;
    boolean grabByClip;

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
        grabByClip = true;

        sm = new StateMachineBuilder()
                .state(State.HOME)
                .onEnter(() -> dt.setDrivePower(1.0))
                .transition(() -> stickyG1.share || stickyG2.share, State.MANUAL_RESET, () -> {
                    gamepad1.rumble(350);
                    gamepad2.rumble(350);
                })
                .transition(() -> (stickyG1.left_bumper || stickyG2.left_bumper) && pivot.getAngle() < 0.2, State.PREINTAKE, () -> {
                    new SequentialCommandGroup(
                            new TurretCenterCommand(),
                            new PreIntakeCommand(),
                            new WaitCommand(140),
                            new ExtensionCommand(15)
                    ).schedule();
                    orientation = spinWrist.setGlobalAngle(SampleOrientation.VERTICAL);
                })
                .transition(() -> stickyG1.y && extension.getDistance() < 2.0, State.SCORING_BASKET, () -> {
                    new SampleBackHighCommand().schedule();
                })
                .transition(() -> stickyG2.x && extension.getDistance() < 2.0, State.INTAKING_SPEC, () -> {
                    if (grabByClip) {
                        new SpecimenIntakeBackClipCommand().schedule();
                    } else {
                        new SpecimenIntakeBackFlatCommand().schedule();
                    }
                })
                .loop(() -> {
                    if(stickyG1.right_bumper || stickyG2.right_bumper) new SpitCommand().schedule();
                })

                .state(State.PREINTAKE)
                .onEnter(() -> {
                    spinWrist.setTurretGlobalAngle(orientation.angle());
                    dt.setDrivePower(0.45);
                })
                .transition(() -> stickyG1.a || stickyG2.a, State.HOME, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> stickyG1.left_bumper || stickyG2.left_bumper, State.GRABBED_GROUND, () -> {
                    new GrabCommand().schedule();
                })

                .loop(() -> {
                    if (stickyG1.left_trigger)
                        turret.setMotionProfileAngle(-1.0);
                    if (stickyG2.left_trigger)
                        turret.setMotionProfileAngle(1.0);
                    if (stickyG1.left_trigger_released || stickyG2.left_trigger_released)
                        turret.center();

                    if(stickyG1.right_bumper || stickyG2.right_bumper)
                        orientation = spinWrist.setGlobalAngle(orientation.next());
                    if (stickyG1.dpad_down || stickyG2.dpad_down)
                        orientation = spinWrist.setGlobalAngle(orientation.prev());
                })

                .state(State.GRABBED_GROUND)
                .transitionTimed(0.63, State.SENSING_GROUND)
                .state(State.SENSING_GROUND)
                .transition(() -> cactus.validSample, State.HOME, () -> {
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
                .transition(() -> stickyG1.a, State.HOME, () -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(80),
                            new RetractFromBasketCommand()
                    ).schedule();
                })

                .state(State.INTAKING_SPEC)
                .onEnter(() -> dt.setDrivePower(0.7))
                .transition(() -> stickyG2.a, State.HOME, () -> new FullRetractCommand().schedule())
                .transition(() -> (stickyG2.left_bumper || cactus.validSample) && pivot.getAngle() > 1.3, State.GRABBED_SPEC, () -> {
                    new SequentialCommandGroup(
                            new ClawGrabCommand(),
                            new WaitCommand(140),
                            new ConditionalCommand(
                                    // raise spec off wall
                                    new SequentialCommandGroup(
                                            new ExtensionCommand(7.0),
                                            new WaitCommand(90),
                                            new UpDownWristAngleCommand(-1.6)
                                    ),
                                    // tilt spec off wall
                                    new SequentialCommandGroup(
                                            new PivotCommand(1.0),
                                            new UpDownWristAngleCommand(-2.0)
                                    ),
                                    () -> grabByClip
                            )
                    ).schedule();
                })

                .state(State.GRABBED_SPEC)
                .transitionTimed(0.3, State.SENSING_SPEC)
                .state(State.SENSING_SPEC)
                .transitionTimed(0.1, State.INTAKING_SPEC, () -> {
                    if(grabByClip) {
                        new SpecimenIntakeBackClipCommand().schedule();
                    } else {
                        new SpecimenIntakeBackFlatCommand().schedule();
                    }
                })
                .transition(() -> cactus.validSample || gamepad2.left_bumper, State.SCORING_SPEC, () -> {
                    new SequentialCommandGroup(
                            new ExtensionCommand(2),
                            new ConditionalCommand(
                                    new SpecimenFrontClipUnderneathCommand(),
                                    new SpecimenFrontFlatCommand(),
                                    () -> grabByClip
                            )
                    ).schedule();
                })

                .state(State.SCORING_SPEC)
                .onEnter(() -> dt.setDrivePower(0.8))
                .transition(() -> stickyG2.left_bumper, State.INTAKING_SPEC, () -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(170),
                            new ExtensionRetractCommand(),
                            new ConditionalCommand(
                                    new SpecimenIntakeBackClipCommand(),
                                    new SpecimenIntakeBackFlatCommand(),
                                    () -> grabByClip
                            )
                    ).schedule();
                })
                .transition(() -> stickyG2.a, State.HOME, () -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new WaitCommand(200),
                            new FullRetractCommand()
                    ).schedule();
                })

                .state(State.MANUAL_RESET)
                .transition(() -> stickyG1.left_bumper || stickyG2.left_bumper, State.HOME, () -> {
                    gamepad1.rumble(150);
                    gamepad2.rumble(150);
                })
                .loop(() -> {
                    double extensionPower;
                    double pivotPower;
                    if (Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad2.left_stick_y))
                        pivotPower = -gamepad1.left_stick_y;
                    else
                        pivotPower = -gamepad2.left_stick_y;

                    if (Math.abs(gamepad1.right_stick_y) > Math.abs(gamepad2.right_stick_y))
                        extensionPower = -gamepad1.right_stick_y;
                    else
                        extensionPower = -gamepad2.right_stick_y;

                    extension.setManualPower(extensionPower);
                    pivot.setManualPower(pivotPower);
                })
                .onExit(() -> {
                    extension.resetEncoder();
                    pivot.resetEncoder();

                    extension.pidTo(0);
                    pivot.pidTo(0);
                })
                .build();

        sm.setState(State.MANUAL_RESET);
        sm.start();
    }

    @Override
    public void periodic() {
        switch (Enum.valueOf(State.class, sm.getStateString())) {
            case MANUAL_RESET:
                dt.drive(new Pose2d(0, 0, 0));
                break;
            default:
                if (gamepad1.getGamepadId() == -1) dt.teleOpDrive(gamepad2);
                else dt.teleOpDrive(gamepad1);
                break;
        }

        if(gamepad1.right_stick_button || gamepad2.right_stick_button) {
            dt.setHeading(Math.PI/2);
            gamepad1.rumble(150);
        }
        sm.update();
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", sm.getState());
        telemetry.addData("Spin wrist angle", orientation);
    }
}
