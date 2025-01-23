package org.firstinspires.ftc.teamcode.blucru.opmode.teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.GetHooksCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.BoxtubeHooksTopBarCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.BoxtubeRetractHang3Command;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.HangServosHangComamnd;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.HangServosReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.HangServosRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.pusher.PushCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleBackLowCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleFrontHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleFrontLowCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmDropToGroundCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmPreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristOppositeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontDunkRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.DriveBase;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Main", group = "1")
public class Main extends BluLinearOpMode {
    enum State {
        RETRACTED,
        EXTENDING_OVER_INTAKE,
        INTAKING_GROUND,
        SCORING_BASKET,
        INTAKING_SPECIMEN,
        ABOVE_SPECIMEN_FRONT,
        ABOVE_SPECIMEN_BACK,
        DUNKING_SPECIMEN_FRONT,
        DUNKING_SPECIMEN_BACK,
        RETRACTING_FROM_SCORING,
        RETRACTING_FROM_INTAKE,
        MANUAL_RESET,

        HANG_RELEASE,
        HANG_2,
        HANG_3,
        HANGING,

//        AUTO_BASKET,
//        AUTO_TO_ASCENT,
//        AUTO_TO_RUNG,
        AUTO_SPECIMEN_INTAKE
    }

    public static double intakeExtendMid = 5, intakeExtendFar = 13;

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
        addHangMotor();
        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());

        sm = new StateMachineBuilder()
                .state(State.RETRACTED)
                .onEnter(() -> dt.setDrivePower(1.0))
                .transition(() -> stickyG2.share, State.MANUAL_RESET, () -> {
                    gamepad1.rumble(350);
                    gamepad2.rumble(350);
                })

                // INTAKE
                .transition(() -> stickyG2.left_bumper, State.INTAKING_GROUND, () -> {
                    new PivotRetractCommand().schedule();
                    new ArmDropToGroundCommand().schedule();
                    new WheelIntakeCommand().schedule();
                    new ClampReleaseCommand().schedule();
                    extension.teleExtendIntake(0);
                })
                .transition(() -> stickyG2.dpad_up, State.EXTENDING_OVER_INTAKE, () -> {
                    new ArmPreIntakeCommand().schedule();
                    new PivotRetractCommand().schedule();
                    new WristUprightForwardCommand().schedule();
                    extension.teleExtendIntake(intakeExtendFar);
                })
                .transition(() -> stickyG2.dpad_right, State.EXTENDING_OVER_INTAKE, () -> {
                    new ArmPreIntakeCommand().schedule();
                    new PivotRetractCommand().schedule();
                    new WristUprightForwardCommand().schedule();
                    extension.teleExtendIntake(intakeExtendMid);
                })

                // LOW
                .transition(() -> stickyG2.b && !gamepad2.dpad_left, State.SCORING_BASKET, () ->
                    new SampleBackLowCommand().schedule())
                .transition(() -> stickyG2.b && gamepad2.dpad_left, State.SCORING_BASKET, () ->
                    new SampleFrontLowCommand().schedule())

                // HIGH
                .transition(() -> stickyG2.y && !gamepad2.dpad_left, State.SCORING_BASKET, () ->
                    new SampleBackHighCommand().schedule())
                .transition(() -> stickyG2.y && gamepad2.dpad_left, State.SCORING_BASKET, () ->
                    new SampleFrontHighCommand().schedule())

                // DRIVE PID
//                .transition(() -> stickyG1.y, State.AUTO_BASKET, () -> {
//                    currentPath = new TeleSampleHighLiftPath().build().start();
//                })


                // SPECIMEN
                .transition(() -> stickyG2.dpad_down, State.INTAKING_SPECIMEN, () -> {
                    new BoxtubeCommand(0.45, 0).schedule();
                    new WristOppositeCommand().schedule();
                    new ArmGlobalAngleCommand(0).schedule();
                })
                .transition(() -> stickyG2.x && !gamepad2.dpad_left, State.ABOVE_SPECIMEN_BACK, () ->
                        new SpecimenBackCommand().schedule())
                .transition(() -> stickyG2.x && gamepad2.dpad_left, State.ABOVE_SPECIMEN_FRONT, () ->
                        new SpecimenFrontCommand().schedule())

                .transition(() -> stickyG1.dpad_down, State.HANG_RELEASE, () -> {
//                    new HangServosReleaseCommand().schedule();
                    new GetHooksCommand().schedule();
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

                .state(State.EXTENDING_OVER_INTAKE)
                .onEnter(() -> dt.setDrivePower(0.75))
                .transition(() -> gamepad2.left_bumper, State.INTAKING_GROUND, () -> {
                    new ArmDropToGroundCommand().schedule();
                    new WheelIntakeCommand().schedule();
                    new ClampReleaseCommand().schedule();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new ExtensionRetractCommand().schedule();
                    new EndEffectorRetractCommand().schedule();
                })
                .loop(() -> {
                    if(stickyG2.dpad_up) extension.teleExtendIntake(intakeExtendFar);
                    if(stickyG2.dpad_right) extension.teleExtendIntake(intakeExtendMid);
                    extension.teleExtendIntakeDelta(-gamepad2.right_stick_y * 4.5);

                    if(gamepad2.right_bumper) {
                        wheel.reverse();
                        clamp.release();
                    } else {
                        wheel.stop();
                        clamp.grab();
                    }
                })

                .state(State.INTAKING_GROUND)
                .onEnter(() -> {
                    dt.setDrivePower(0.55);
                    wheel.intake();
                    clamp.release();
                })
                .transition(() -> stickyG2.a || intakeSwitch.pressed(), State.RETRACTED, () -> {
                    if(intakeSwitch.pressed()) {
                        gamepad1.rumble(200);
                        gamepad2.rumble(200);
                    }

                    new SequentialCommandGroup(
                            new EndEffectorRetractCommand(),
                            new WaitCommand(100),
                            new BoxtubeRetractCommand()
                    ).schedule();
                })
                .transition(() -> !gamepad2.left_bumper, State.EXTENDING_OVER_INTAKE, () -> {
                    new ClampGrabCommand().schedule();
                    new WheelStopCommand().schedule();
                    new ArmPreIntakeCommand().schedule();
                })
                .loop(() -> {
                    if(stickyG2.dpad_up) extension.teleExtendIntake(intakeExtendFar);
                    if(stickyG2.dpad_right) extension.teleExtendIntake(intakeExtendMid);

                    extension.teleExtendIntakeDelta(-gamepad2.right_stick_y * 4.5);
                })

                .onExit(() -> {
                    wheel.stop();
                    clamp.close();
                })

                .state(State.INTAKING_SPECIMEN)
                .onEnter(() -> dt.setDrivePower(0.4))
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ArmRetractCommand(),
                            new PivotCommand(1),
                            new WaitCommand(300),
                            new BoxtubeRetractCommand(),
                            new EndEffectorRetractCommand()
                    ).schedule();
                })
                .transition(() -> stickyG2.x && !gamepad2.dpad_left, State.ABOVE_SPECIMEN_BACK, () -> {
                    new BoxtubeSplineCommand(
                            new Vector2d(20,42),
                            new Pose2d(-8.6, 30, Math.PI),
                            0,
                            0.75
                    ).schedule();
                })
                .transition(() -> stickyG2.x && gamepad2.dpad_left, State.ABOVE_SPECIMEN_FRONT, () -> {
                    new SequentialCommandGroup(
                            new ArmGlobalAngleCommand(1.2),
                            new PivotCommand(1),
                            new WaitCommand(300),
                            new BoxtubeCommand(1.4, 5),
                            new WristOppositeCommand(),
                            new ArmGlobalAngleCommand(0.64)
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

                .state(State.ABOVE_SPECIMEN_FRONT)
                .onEnter(() -> dt.setDrivePower(0.5))
                .transition(() -> gamepad2.left_bumper, State.DUNKING_SPECIMEN_FRONT, () ->
                        new SpecimenFrontDunkCommand().schedule())
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ArmGlobalAngleCommand(1.2),
                            new WaitCommand(200),
                            new EndEffectorRetractCommand(),
                            new BoxtubeRetractCommand()
                    ).schedule();
                })
                .transition(() -> stickyG2.x && !gamepad2.dpad_left, State.ABOVE_SPECIMEN_BACK, () -> {
                    new SpecimenBackCommand().schedule();
                })

                .state(State.DUNKING_SPECIMEN_FRONT)
                .onEnter(() -> dt.setDrivePower(0.4))
                .transition(() -> !gamepad2.left_bumper, State.ABOVE_SPECIMEN_FRONT, () -> {
                    new SpecimenFrontCommand().schedule();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SpecimenFrontDunkRetractCommand().schedule();
                })

                .state(State.ABOVE_SPECIMEN_BACK)
                .onEnter(() -> dt.setDrivePower(0.55))
                .transition(() -> stickyG2.left_bumper, State.DUNKING_SPECIMEN_BACK,
                        () -> new BoxtubeSplineCommand(
                                new Pose2d(-9, 25.6, Math.PI),
                                new Vector2d(-8, -1.5),
                                new Pose2d(-9.271, 21.681, Math.PI),
                                new Vector2d(0,0),
                                0,
                                0.35
                        ).schedule())

                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new PivotCommand(0.9),
                            new WaitCommand(200),
                            new EndEffectorRetractCommand(),
                            new BoxtubeRetractCommand()
                    ).schedule();
                })
                .transition(() -> stickyG2.x && gamepad2.dpad_left, State.ABOVE_SPECIMEN_FRONT,
                        () -> new SpecimenFrontCommand().schedule())

                .state(State.DUNKING_SPECIMEN_BACK)
                .onEnter(() -> dt.setDrivePower(0.35))
                .transition(() -> !gamepad2.left_bumper, State.ABOVE_SPECIMEN_BACK, () ->
                        new SpecimenBackCommand().schedule())

                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SpecimenBackDunkRetractCommand().schedule();
                })

                .state(State.SCORING_BASKET)
                .onEnter(() -> dt.setDrivePower(0.35))
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ArmGlobalAngleCommand(1.5),
                            new WaitCommand(150),
                            new BoxtubeRetractCommand(),
                            new EndEffectorRetractCommand()
                    ).schedule();
                })
//                .transition(() -> stickyG1.y, State.AUTO_TO_ASCENT, () -> {
//                    currentPath = new TeleDriveToAscentPath().build().start();
//                })
//                .transition(() -> stickyG1.b, State.AUTO_TO_RUNG, () -> {
//                    currentPath = new TeleDriveToRungIntakePath().build().start();
//                })
                .loop(() -> {
                    if(gamepad2.left_bumper) {
                        clamp.release();
                        wheel.setPower(-0.5);
                    } else {
                        clamp.grab();
                        wheel.stop();
                    }

                    if(stickyG2.y && !gamepad2.dpad_left) {
                        new SampleBackHighCommand().schedule();
                    } else if(stickyG2.y && gamepad2.dpad_left) {
                        new SampleFrontHighCommand().schedule();
                    } else if(stickyG2.b && !gamepad2.dpad_left) {
                        new SampleBackLowCommand().schedule();
                    } else if(stickyG2.b && gamepad2.dpad_left) {
                        new SampleFrontLowCommand().schedule();
                    }
                })
                .onExit(() -> {
                    clamp.grab();
                    wheel.stop();
                })

//                .state(State.AUTO_BASKET)
//                .transition(() ->
//                    Math.abs(gamepad1.left_stick_y) > 0.1
//                        || Math.abs(gamepad1.left_stick_x) > 0.1
//                        || Math.abs(gamepad1.right_stick_x) > 0.1,
//                    State.SCORING_BASKET, () -> {
//                    currentPath.cancel();
//                    new SampleBackHighCommand().schedule();
//                })
//                .transition(() -> currentPath.isDone() || stickyG2.left_bumper, State.SCORING_BASKET)
//                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
//                    currentPath.cancel();
//                    new FullRetractCommand().schedule();
//                })
//
//                .state(State.AUTO_TO_ASCENT)
//                .transition(() ->
//                        Math.abs(gamepad1.left_stick_y) > 0.1
//                                || Math.abs(gamepad1.left_stick_x) > 0.1
//                                || Math.abs(gamepad1.right_stick_x) > 0.1
//                                || currentPath.isDone(),
//                        State.RETRACTED, () -> {
//                            currentPath.cancel();
//                            new FullRetractCommand().schedule();
//                        })
//
//                .state(State.AUTO_TO_RUNG)
//                .transition(() ->
//                                Math.abs(gamepad1.left_stick_y) > 0.1
//                                        || Math.abs(gamepad1.left_stick_x) > 0.1
//                                        || Math.abs(gamepad1.right_stick_x) > 0.1
//                                        || currentPath.isDone(),
//                        State.RETRACTED, () -> {
//                            currentPath.cancel();
//                            new FullRetractCommand().schedule();
//                        })

                .state(State.HANG_RELEASE)
                .onEnter(() -> {
                    dt.setDrivePower(0.55);
                })
                .transition(() -> stickyG1.dpad_down, State.HANG_2, () -> {
                    new SequentialCommandGroup(
                            new HangServosHangComamnd(),
                            new WaitCommand(200),
                            new BoxtubeHooksTopBarCommand()
                    ).schedule();
                })
                .transition(() -> stickyG1.dpad_up, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                    new HangServosRetractCommand().schedule();
                })

                .state(State.HANG_2)
                .transition(() -> stickyG1.dpad_down, State.HANG_3, () -> {
                    new SequentialCommandGroup(
                            new WaitCommand(1000),
                            new BoxtubeRetractHang3Command()
                    ).schedule();
                })
                .transition(() -> stickyG1.dpad_up, State.HANG_RELEASE, () -> {
                    new HangServosReleaseCommand().schedule();
                    new BoxtubeSplineCommand(
                            new Pose2d(2.5, 36, 1.5),
                            -Math.PI/2,
                            0.7
                    ).schedule();
                })

                .state(State.HANG_3)
                .transition(() -> stickyG1.dpad_up, State.HANG_2, () -> {
                    new SequentialCommandGroup(
                            new PivotCommand(1.5),
                            new WaitCommand(700),
                            new BoxtubeHooksTopBarCommand()
                    ).schedule();
                })
                .transition(() -> stickyG1.dpad_down, State.HANGING, () -> {
                    new FullRetractCommand().schedule();
                    // TODO: new HangPID hold position command
                })

                .state(State.HANGING)
                .transition(() -> stickyG1.right_stick_button || stickyG2.right_stick_button, State.RETRACTED, () -> {
                    new HangServosRetractCommand().schedule();
                    new FullRetractCommand().schedule();
                })

                .state(State.MANUAL_RESET)
                .onEnter(() -> {
                    dt.setDrivePower(0.8);
                })
                .transition(() -> stickyG2.left_bumper || stickyG1.left_bumper, State.RETRACTED, () -> {
                    gamepad1.rumble(150);
                    gamepad2.rumble(150);
                })
                .loop(() -> {
                    extension.setManualPower(-gamepad2.right_stick_y);
                    pivot.setManualPower(-gamepad2.left_stick_y);
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
    public void onStart() {
        pivot.pidTo(0);
        extension.pidTo(0);

        dt.setPoseEstimate(DriveBase.startPose);
    }

    @Override
    public void periodic() {
        switch (Enum.valueOf(State.class, sm.getStateString())) {
//            case AUTO_BASKET:
//            case AUTO_TO_ASCENT:
//            case AUTO_TO_RUNG:
//                currentPath.run();
//                break;
            case HANG_2:
            case HANG_3:
                hangMotor.setManualPower(-gamepad1.right_stick_y);
                dt.drive(new Pose2d(0,0,0));
                break;
            case HANGING:
                hangMotor.setManualPower(-0.75);
                break;
            default:
                if(gamepad1.a)
                    dt.driveToHeading(gamepad1.left_stick_x, -gamepad1.left_stick_y, Math.PI/4);
                else
                    dt.teleOpDrive(gamepad1);
        }

        if(gamepad1.right_stick_button) {
            dt.setHeading(Math.PI/2);
            gamepad1.rumble(150);
        }

        if(stickyG1.right_bumper) {
            new PushCommand().schedule();
        }

//        if(stickyG1.left_bumper) hangServos.toggle();
        sm.update();
    }

    @Override
    public void telemetry() {
        telemetry.addData("state", sm.getState());
    }
}
