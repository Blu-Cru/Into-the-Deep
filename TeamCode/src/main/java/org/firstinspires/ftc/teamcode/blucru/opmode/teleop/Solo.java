package org.firstinspires.ftc.teamcode.blucru.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmDropToGroundCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmPreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristOppositeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.pusher.PushCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleBackLowCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleFrontHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleFrontLowCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.tele.TeleDriveToAscentPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.tele.TeleDriveToRungIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.DriveBase;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "2")
public class Solo extends BluLinearOpMode {
    enum State {
        RETRACTED,
        EXTENDING_OVER_INTAKE,
        INTAKING_GROUND,
        SCORING_BASKET,
        INTAKING_SPECIMEN,
        ABOVE_SPECIMEN_BACK,
        DUNKING_SPECIMEN_BACK,
        MANUAL_RESET,

        AUTO_BASKET,
        AUTO_TO_ASCENT,
        AUTO_TO_RUNG,
        AUTO_SPECIMEN_INTAKE
    }

    StateMachine sm;
    Path currentPath;

    @Override
    public void initialize() {
        addDrivetrain();
        addExtension();
        addPivot();
        addArm();
        addWheel();
        addClamp();
        addWrist();
        addIntakeSwitch();
        addPusher();
        addHangServos();
        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());

        sm = new StateMachineBuilder()
                .state(State.RETRACTED)
                .onEnter(() -> dt.setDrivePower(1.0))
                .transition(() -> stickyG1.share, State.MANUAL_RESET, () -> gamepad1.rumble(350))

                // INTAKE
                .transition(() -> gamepad1.left_bumper, State.INTAKING_GROUND, () -> {
                    new PivotRetractCommand().schedule();
                    new ArmDropToGroundCommand().schedule();
                    new WheelIntakeCommand().schedule();
                    new ClampReleaseCommand().schedule();
                    extension.teleExtendIntake(0);
                })

                // LOW
                .transition(() -> stickyG1.b, State.SCORING_BASKET, () ->
                        new SampleBackLowCommand().schedule())
                // HIGH
                .transition(() -> stickyG1.y, State.SCORING_BASKET, () ->
                        new SampleBackHighCommand().schedule())

                // SPECIMEN
                .transition(() -> stickyG2.x, State.INTAKING_SPECIMEN, () -> {
                    new BoxtubeCommand(0.42, 0).schedule();
                    new WristOppositeCommand().schedule();
                    new ArmGlobalAngleCommand(0).schedule();
                })

                .loop(() -> {
                    if(stickyG2.right_bumper) {
                        new SequentialCommandGroup(
                                new ArmPreIntakeCommand(),
                                new WaitCommand(300),
                                new ClampReleaseCommand(),
                                new WheelReverseCommand(),
                                new WaitCommand(100),
                                new EndEffectorRetractCommand()
                        ).schedule();
                    }
                })

                .state(State.INTAKING_GROUND)
                .onEnter(() -> {
                    dt.setDrivePower(0.4);
                    wheel.intake();
                    clamp.release();
                })
                .transition(() -> stickyG1.a || intakeSwitch.pressed(), State.RETRACTED, () -> {
                    if(intakeSwitch.pressed()) {
                        gamepad1.rumble(200);
                    }

                    new SequentialCommandGroup(
                            new ClampGrabCommand(),
                            new WheelStopCommand(),
                            new ArmPreIntakeCommand(),
                            new WristUprightForwardCommand(),
                            new WaitCommand(100),
                            new ArmRetractCommand(),
                            new BoxtubeRetractCommand()
                    ).schedule();
                })
                .transition(() -> !gamepad1.left_bumper, State.EXTENDING_OVER_INTAKE, () -> {
                    new ClampGrabCommand().schedule();
                    new WheelStopCommand().schedule();
                    new ArmPreIntakeCommand().schedule();
                })
                .loop(() -> {
                    if(stickyG1.b) extension.teleExtendIntake(Main.intakeExtendFar);
                    if(stickyG1.y) extension.teleExtendIntake(Main.intakeExtendMid);

                    extension.teleExtendIntakeDelta(gamepad1.right_trigger * 6);
                })
                .onExit(() -> {
                    wheel.stop();
                    clamp.close();
                })

                .state(State.EXTENDING_OVER_INTAKE)
                .onEnter(() -> dt.setDrivePower(0.7))
                .transition(() -> gamepad1.left_bumper, State.INTAKING_GROUND, () -> {
                    new ArmDropToGroundCommand().schedule();
                    new WheelIntakeCommand().schedule();
                    new ClampReleaseCommand().schedule();
                })
                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .loop(() -> {
                    if(stickyG1.b) extension.teleExtendIntake(Main.intakeExtendFar);
                    if(stickyG1.y) extension.teleExtendIntake(Main.intakeExtendMid);

                    extension.teleExtendIntakeDelta(gamepad1.right_trigger * 6);

                    if(gamepad1.right_bumper) {
                        wheel.reverse();
                        clamp.release();
                    } else {
                        wheel.stop();
                        clamp.grab();
                    }
                })

                .state(State.INTAKING_SPECIMEN)
                .onEnter(() -> dt.setDrivePower(0.4))
                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ArmRetractCommand(),
                            new PivotCommand(1),
                            new WaitCommand(300),
                            new BoxtubeRetractCommand(),
                            new EndEffectorRetractCommand()
                    ).schedule();
                })
                .transition(() -> stickyG1.x, State.ABOVE_SPECIMEN_BACK, () -> {
                    new BoxtubeSplineCommand(
                            new Vector2d(20,42),
                            new Pose2d(-8.6, 30, Math.PI),
                            0,
                            0.75
                    ).schedule();
                })
                .loop(() -> {
                    if(gamepad2.left_bumper) {
                        clamp.release();
                        wheel.intake();
                    } else if(gamepad2.right_bumper) {
                        clamp.release();
                        wheel.reverse();
                    } else {
                        clamp.grab();
                        wheel.stop();
                    }
                })
                .onExit(() -> {
                    clamp.grab();
                    wheel.stop();
                })

                .state(State.ABOVE_SPECIMEN_BACK)
                .onEnter(() -> dt.setDrivePower(0.55))
                .transition(() -> stickyG1.left_bumper, State.DUNKING_SPECIMEN_BACK,
                        () -> new SpecimenBackDunkCommand().schedule())

                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new PivotCommand(0.9),
                            new WaitCommand(150),
                            new EndEffectorRetractCommand(),
                            new BoxtubeRetractCommand()
                    ).schedule();
                })

                .state(State.DUNKING_SPECIMEN_BACK)
                .onEnter(() -> dt.setDrivePower(0.35))
                .transition(() -> !gamepad1.left_bumper, State.ABOVE_SPECIMEN_BACK, () ->
                        new SpecimenBackCommand().schedule())

                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
                    new SpecimenBackDunkRetractCommand().schedule();
                })

                .state(State.SCORING_BASKET)
                .onEnter(() -> dt.setDrivePower(0.35))
                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ArmGlobalAngleCommand(1.5),
                            new WaitCommand(150),
                            new BoxtubeRetractCommand(),
                            new EndEffectorRetractCommand()
                    ).schedule();
                })
                .loop(() -> {
                    if(gamepad1.left_bumper) {
                        clamp.release();
                        wheel.reverse();
                    } else {
                        clamp.grab();
                        wheel.stop();
                    }

                    if(stickyG1.y) new SampleBackHighCommand().schedule();
                    else if(stickyG1.b) new SampleBackLowCommand().schedule();
                })
                .onExit(() -> {
                    clamp.grab();
                    wheel.stop();
                })

                .state(State.MANUAL_RESET)
                .onEnter(() -> dt.setDrivePower(0.8))
                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
                    gamepad1.rumble(150);
                })
                .loop(() -> {
                    extension.setManualPower(-gamepad1.right_stick_y);
                    pivot.setManualPower(-gamepad1.left_stick_y);
                })
                .onExit(() -> {
                    extension.resetEncoder();
                    pivot.resetEncoder();

                    extension.pidTo(0);
                    pivot.pidTo(0);
                })

                .build();
    }

    @Override
    public void onStart() {
        pivot.pidTo(0);
        extension.pidTo(0);

        dt.setPoseEstimate(DriveBase.startPose);
    }

    @Override
    public void periodic() {
        switch (Enum.valueOf(State.class, sm.getStateString())) {
            case AUTO_BASKET:
            case AUTO_TO_ASCENT:
            case AUTO_TO_RUNG:
                currentPath.run();
                break;
            case MANUAL_RESET:
                break;
            default:
                dt.teleOpDrive(gamepad1);
        }

        if(gamepad1.right_stick_button) {
            dt.setHeading(Math.PI/2);
            gamepad1.rumble(150);
        }

        if(stickyG1.left_trigger) {
            new PushCommand().schedule();
        }

        if(stickyG1.left_bumper) hangServos.toggle();
        sm.update();
    }

    @Override
    public void telemetry() {
        telemetry.addData("state", sm.getState());
    }
}
