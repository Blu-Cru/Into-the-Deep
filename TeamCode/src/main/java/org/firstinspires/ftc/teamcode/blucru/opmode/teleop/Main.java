package org.firstinspires.ftc.teamcode.blucru.opmode.teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristHorizontalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.GetHooksHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.BoxtubeHooksTopBarCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.HangServosHangComamnd;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.HangServosReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.hang.HangServosRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.pusher.PushCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleBackLowCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleFrontHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleFrontLowCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkRetractCommand;
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
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenDunkSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.SpecimenCycleIntakeFailsafePath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.SpecimenIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.tele.TeleDriveToAscentPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.tele.TeleDriveToRungIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.tele.TeleSampleHighLiftPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.tele.TeleSpecimenDepoPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.DriveBase;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Main", group = "1")
public class Main extends BluLinearOpMode {
    enum State {
        RETRACTED,
        EXTENDING_OVER_INTAKE,
        INTAKING_GROUND,
        SCORING_BASKET,
        INTAKING_SPECIMEN,
        ABOVE_SPECIMEN_BACK,
        DUNKING_SPECIMEN_BACK,
        MANUAL_RESET,

        HANG_RELEASE,
        HANG_HOOKS_ON_BAR,
        HANG_BOXTUBE_EXTENDED,
        HANG_PULLING_ABOVE_BAR,
        HANGING,

        AUTO_SAMPLE_BASKET,
        AUTO_SAMPLE_TO_ASCENT,
        AUTO_SAMPLE_TO_RUNG,

        AUTO_SPEC_INTAKE,
        AUTO_SPEC_INTAKE_FAIL,
        AUTO_SPEC_DEPO_PATH,
        AUTO_SPEC_DEPO_MANUAL,
        AUTO_SPEC_SCORING
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
        addCVMaster();
        addCactus();
        enableFTCDashboard();
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
                .transition(() -> stickyG1.y && cvMaster.seesSampleTag(), State.AUTO_SAMPLE_BASKET, () -> {
                    dt.updateAprilTags();
                    currentPath = new TeleSampleHighLiftPath().build().start();
                })

                // SPECIMEN
                .transition(() -> stickyG2.dpad_down, State.INTAKING_SPECIMEN, () -> {
                    new BoxtubeCommand(0.45, 0).schedule();
                    new WristOppositeCommand().schedule();
                    new ArmGlobalAngleCommand(0).schedule();
                })
                .transition(() -> stickyG2.x, State.ABOVE_SPECIMEN_BACK, () ->
                    new SpecimenBackCommand().schedule())

                .transition(() -> stickyG1.dpad_down, State.HANG_RELEASE, () -> {
                    new GetHooksHighCommand().schedule();
                })

                // AUTO SPEC
                .transition(() -> stickyG1.b && cvMaster.seesSpecimenTag(), State.AUTO_SPEC_INTAKE, () -> {
                    dt.updateAprilTags();
                    currentPath = new SpecimenIntakePath().start();
                })
//                .transition(() -> stickyG1.b && cvMaster.seesSampleTag(), State.AUTO_SAMPLE_TO_ASCENT, () -> {
//                    currentPath = new TeleDriveToAscentPath().build().start();
//                })

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
                .transition(() -> stickyG2.a || Robot.validSample(), State.RETRACTED, () -> {
                    if(Robot.validSample()) {
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
                .transition(() -> stickyG2.x || Robot.validSpecimen(), State.ABOVE_SPECIMEN_BACK, () -> {
                    new BoxtubeSplineCommand(
                            new Vector2d(20,42),
                            new Pose2d(-8.6, 30, Math.PI),
                            0,
                            0.75
                    ).schedule();

                    if(Robot.validSpecimen()) {
                        gamepad1.rumble(200);
                        gamepad2.rumble(200);
                    }
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
                .transition(() -> stickyG2.left_bumper, State.DUNKING_SPECIMEN_BACK,
                        () -> new BoxtubeSplineCommand(
                                new Pose2d(-9, 25.8, Math.PI),
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
                .transition(() -> stickyG1.y, State.AUTO_SAMPLE_TO_ASCENT, () -> {
                    currentPath = new TeleDriveToAscentPath().build().start();
                })
                .transition(() -> stickyG1.b, State.AUTO_SAMPLE_TO_RUNG, () -> {
                    currentPath = new TeleDriveToRungIntakePath().build().start();
                })
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

                .state(State.AUTO_SAMPLE_BASKET)
                .transition(() ->
                    Math.abs(gamepad1.left_stick_y) > 0.1
                        || Math.abs(gamepad1.left_stick_x) > 0.1
                        || Math.abs(gamepad1.right_stick_x) > 0.1,
                    State.SCORING_BASKET, () -> {
                    currentPath.cancel();
                    new SampleBackHighCommand().schedule();
                })
                .transition(() -> currentPath.isDone() || stickyG2.left_bumper, State.SCORING_BASKET)
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    currentPath.cancel();
                    new FullRetractCommand().schedule();
                })

                .state(State.AUTO_SAMPLE_TO_ASCENT)
                .transition(() ->
                        Math.abs(gamepad1.left_stick_y) > 0.1
                                || Math.abs(gamepad1.left_stick_x) > 0.1
                                || Math.abs(gamepad1.right_stick_x) > 0.1
                                || currentPath.isDone(),
                        State.RETRACTED, () -> {
                            currentPath.cancel();
                            new FullRetractCommand().schedule();
                        })

                .state(State.AUTO_SAMPLE_TO_RUNG)
                .transition(() ->
                                Math.abs(gamepad1.left_stick_y) > 0.1
                                        || Math.abs(gamepad1.left_stick_x) > 0.1
                                        || Math.abs(gamepad1.right_stick_x) > 0.1
                                        || currentPath.isDone(),
                        State.RETRACTED, () -> {
                            currentPath.cancel();
                            new FullRetractCommand().schedule();
                        })

                .state(State.AUTO_SPEC_INTAKE)
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> currentPath.isDone(), State.AUTO_SPEC_INTAKE_FAIL, () -> {
                    currentPath = new SpecimenCycleIntakeFailsafePath().start();
                })
                .transition(() -> Robot.getInstance().intakeSwitch.justPressed() && pivot.getAngle() < 0.5, State.AUTO_SPEC_DEPO_PATH, () -> {
                    currentPath = new TeleSpecimenDepoPath().start();
                })

                .state(State.AUTO_SPEC_DEPO_PATH)
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> currentPath.isDone(), State.AUTO_SPEC_DEPO_MANUAL, () -> {
                    dt.setDrivePower(0.7);
                    dt.idle();
                })

                .state(State.AUTO_SPEC_DEPO_MANUAL)
                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> stickyG1.b, State.AUTO_SPEC_SCORING, () -> {
                    dt.pidTo(dt.pose);
                    new SequentialCommandGroup(
                            new SpecimenDunkSplineCommand(),
                            new WaitCommand(280),
                            new WheelReverseCommand(),
                            new ClampReleaseCommand(),
                            new WaitCommand(150),
                            new PivotRetractCommand(),
                            new ExtensionRetractCommand(),
                            new WaitCommand(100),
                            new FullRetractCommand()
                    ).schedule();
                })

                .state(State.AUTO_SPEC_SCORING)
                .transitionTimed(0.25, State.AUTO_SPEC_INTAKE, () -> {
                    currentPath = new SpecimenIntakePath().start();
                })

                .state(State.AUTO_SPEC_INTAKE_FAIL)
                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> Robot.getInstance().intakeSwitch.justPressed(), State.AUTO_SPEC_DEPO_PATH, () -> {
                    currentPath = new TeleSpecimenDepoPath().start();
                })
                .transition(() -> currentPath.isDone(), State.AUTO_SPEC_INTAKE, () -> {
                    currentPath = new SpecimenIntakePath().start();
                })

                .state(State.HANG_RELEASE) // 1st stage released, hook is up, not touching bar
                .onEnter(() -> {
                    gamepad1.rumble(150);
                    dt.setDrivePower(0.55);
                })
                .transition(() -> stickyG1.dpad_down, State.HANG_HOOKS_ON_BAR, () -> {
                    gamepad1.rumble(150);
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

                .state(State.HANG_HOOKS_ON_BAR) // hooks are on top bar
                .transition(() -> stickyG1.dpad_down, State.HANG_BOXTUBE_EXTENDED, () -> {
                    gamepad1.rumble(150);
                    new SequentialCommandGroup(
                            new PivotCommand(1.5),
                            new WaitCommand(200),
                            new FullRetractCommand(),
                            new WaitCommand(400),
                            new WristHorizontalCommand(),
                            new ExtensionCommand(18)
                    ).schedule();
                })
                .transition(() -> stickyG1.dpad_up, State.HANG_RELEASE, () -> {
                    new HangServosReleaseCommand().schedule();
                    new BoxtubeSplineCommand(
                            new Pose2d(2.5, 36, Math.PI/4),
                            -Math.PI/2,
                            0.7
                    ).schedule();
                })

                .state(State.HANG_BOXTUBE_EXTENDED)
                .transition(() -> stickyG1.dpad_down, State.HANG_PULLING_ABOVE_BAR, () -> {
                    gamepad1.rumble(150);
                    new ExtensionRetractCommand().schedule();
                    pivot.idle();
                })
                .transition(() -> stickyG1.dpad_up, State.HANG_HOOKS_ON_BAR, () -> {
                    new SequentialCommandGroup(
                            new FullRetractCommand(),
                            new WaitCommand(200),
                            new BoxtubeHooksTopBarCommand()
                    ).schedule();
                })

                .state(State.HANG_PULLING_ABOVE_BAR) // pulling up
                .onEnter(() -> {
                    wrist.disable();
                    clamp.disable();
                    pusher.disable();
                })
                .transition(() -> stickyG1.dpad_up, State.HANG_BOXTUBE_EXTENDED, () -> {
                    new SequentialCommandGroup(
                            new FullRetractCommand(),
                            new WaitCommand(500),
                            new ExtensionCommand(18)
                    ).schedule();
                })
                .transition(() -> stickyG1.dpad_down, State.HANGING, () -> {
                    gamepad1.rumble(150);
                    new FullRetractCommand().schedule();
                })
                .onExit(() -> {
                    wrist.enable();
                    clamp.enable();
                    pusher.enable();
                })

                .state(State.HANGING)
                .onEnter(() -> hangMotor.holdPosition())
                .transition(() -> stickyG1.right_stick_button || stickyG2.right_stick_button, State.RETRACTED, () -> {
                    new HangServosRetractCommand().schedule();
                    new FullRetractCommand().schedule();
                })
                .onExit(() -> hangMotor.idle())

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

        cvMaster.detectTag();
    }

    @Override
    public void periodic() {
        switch (Enum.valueOf(State.class, sm.getStateString())) {
            case AUTO_SAMPLE_BASKET:
            case AUTO_SAMPLE_TO_ASCENT:
            case AUTO_SAMPLE_TO_RUNG:
            case AUTO_SPEC_INTAKE:
            case AUTO_SPEC_DEPO_PATH:
            case AUTO_SPEC_INTAKE_FAIL:
                currentPath.run();
                break;
            case AUTO_SPEC_SCORING:
                break;
            case AUTO_SPEC_DEPO_MANUAL:
                dt.pidYHeadingMapped(gamepad1.left_stick_x, -34 - gamepad1.left_stick_y * 5.5, -Math.PI/2);
                break;
            case HANG_HOOKS_ON_BAR:
            case HANG_BOXTUBE_EXTENDED:
            case HANG_PULLING_ABOVE_BAR:
                hangMotor.setManualPower(-gamepad1.right_stick_y);
                dt.drive(new Pose2d(0,0,0));
                break;
            case HANGING:
                dt.drive(new Pose2d(0,0,0));
                break;
            default:
                if(gamepad1.a)
                    dt.driveToHeading(gamepad1.left_stick_x, -gamepad1.left_stick_y, Math.PI/4);
                else
                    dt.teleOpDrive(gamepad1);
        }

        if(gamepad1.right_stick_button) {
            dt.setHeading(Math.PI/2, Globals.alliance);
            gamepad1.rumble(150);
        }

        if(stickyG1.right_bumper) {
            new PushCommand().schedule();
        }

        sm.update();
    }

    @Override
    public void telemetry() {
        telemetry.addData("state", sm.getState());
        dt.drawPose();
    }
}
