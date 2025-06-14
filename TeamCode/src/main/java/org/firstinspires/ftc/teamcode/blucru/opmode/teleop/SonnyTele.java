package org.firstinspires.ftc.teamcode.blucru.opmode.teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.RetractFromBasketCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmSampleScorePositionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.GetHooksHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.BoxtubeHooksTopBarCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.motor.HangMotorHighBarCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.servo.HangServosHangComamnd;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.servo.HangServosReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.hang.servo.HangServosRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.pusher.PushCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.sample.SampleBackHighLiftCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.sample.SampleBackLowCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenBackDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenBackDunkRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenBackCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenDunkSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenCycleIntakeFailsafePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.tele.TeleSpecimenDepoPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.DriveBase;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Sonny", group = "2")
public class SonnyTele extends BluLinearOpMode {
    enum State {
        RETRACTED,
        MANUAL_RESET,

        EXTENDED_OVER_INTAKE_CLOSE,
        EXTENDED_OVER_INTAKE_FAR,
        INTAKING_CLOSE,
        INTAKING_FAR,

        SPITTING_RETRACT,
        SPITTING_FAR,

        LIFTING_BASKET_HIGH,
        SCORING_BASKET_HIGH,
        SCORING_BASKET_LOW,

        INTAKING_SPECIMEN,
        ABOVE_SPECIMEN_BACK,
        DUNKING_SPECIMEN_BACK,

        HANG_RELEASE,
        HANG_HOOKS_ON_BAR,
        HANG_BOXTUBE_EXTENDED,
        HANG_PULLING_ABOVE_BAR,
        HANG_HOOKS_ON_LOW,
        HANG_PULLING_LOW,
        HANGING,

//        AUTO_SAMPLE_LIFTING,
//        AUTO_SAMPLE_SCORING,
//        AUTO_SAMPLE_TO_ASCENT,
//        AUTO_SAMPLE_TO_RUNG,

        AUTO_SPEC_INTAKE,
        AUTO_SPEC_INTAKE_FAIL,
        AUTO_SPEC_DEPO_PATH,
        AUTO_SPEC_DEPO_MANUAL,
        AUTO_SPEC_SCORING
    }

    public static double intakeExtendMid = 6, intakeExtendFar = 17;

    StateMachine sm;
    Path currentPath;

    @Override
    public void initialize() {
        addCVMaster();
        addDrivetrain();
        addExtension();
        addPivot();
        addArm();
        addClaw();
        addPusher();
        addHangServos();
        addHangMotor();
        addCactus();
        enableFTCDashboard();
        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());

//        dt.fieldCentric = false;

        sm = new StateMachineBuilder()
                .state(State.RETRACTED)
                .onEnter(() -> dt.setDrivePower(1.0))
                .transition(() -> stickyG1.share || stickyG2.share, State.MANUAL_RESET, () -> gamepad1.rumble(350))

                // INTAKE
                .transition(() -> stickyG1.left_bumper && pivot.getAngle() < 0.2, State.INTAKING_CLOSE, () -> {
                    new PivotRetractCommand().schedule();
//                    new ArmDropToGroundCommand().schedule();
//                    new WristUprightForwardCommand().schedule();
//                    new WheelIntakeCommand().schedule();
                    new ClawOpenCommand().schedule();
                    extension.teleExtendIntake(0);
                })
                .transition(() -> stickyG1.right_trigger && pivot.getAngle() < 0.2, State.EXTENDED_OVER_INTAKE_FAR, () -> {
                    new PivotRetractCommand().schedule();
//                    new ArmPreIntakeCommand().schedule();
//                    new WristUprightForwardCommand().schedule();
//                    new WheelStopCommand().schedule();
                    new ClawGrabCommand().schedule();
                    extension.teleExtendIntake(intakeExtendFar);
                })

                // sample
                .transition(() -> stickyG1.right_bumper && extension.getDistance() < 3, State.LIFTING_BASKET_HIGH, () -> {
                    new SampleBackHighLiftCommand().schedule();
                })

                // SPECIMEN
                .transition(() -> stickyG1.b && cvMaster.seesSpecimenTag(), State.AUTO_SPEC_INTAKE, () -> {
                    dt.updateAprilTags();
                    currentPath = new SpecimenIntakePath().start();
                })

                // SPECIMEN
                .transition(() -> stickyG1.x, State.INTAKING_SPECIMEN, () -> {
                    new BoxtubeCommand(0.42, 0).schedule();
//                    new WristOppositeCommand().schedule();
                    new ArmGlobalAngleCommand(0).schedule();
                })

                // HANG
                .transition(() -> stickyG1.dpad_down && pivot.getAngle() < 0.2, State.HANG_RELEASE, () -> {
                    new GetHooksHighCommand().schedule();
                })

                .loop(() -> {
                    if(stickyG1.a) {
                        new SequentialCommandGroup(
//                                new ArmPreIntakeCommand(),
//                                new WaitCommand(300),
//                                new ClampReleaseCommand(),
//                                new WheelReverseCommand(),
                                new WaitCommand(100),
                                new EndEffectorRetractCommand()
                        ).schedule();
                    }
                })

                .state(State.INTAKING_CLOSE)
                .onEnter(() -> {
                    dt.setDrivePower(0.6);
                    claw.release();
                })
                .transition(() -> stickyG1.right_trigger, State.INTAKING_FAR, () -> {
                    extension.teleExtendIntake(intakeExtendFar);
                })
                .transition(() -> stickyG1.right_bumper || Robot.justValidSample(), State.RETRACTED, () -> {
                    if(Robot.justValidSample()) {
                        gamepad1.rumble(200);
                    }

                    new FullRetractCommand().schedule();
                })
                .transition(() -> !gamepad1.left_bumper, State.EXTENDED_OVER_INTAKE_CLOSE, () -> {
                    new ClawGrabCommand().schedule();
//                    new WheelStopCommand().schedule();
//                    new ArmPreIntakeCommand().schedule();
                })
                .transition(() -> stickyG1.a, State.SPITTING_RETRACT)
                .onExit(() -> {
                    claw.close();
                })

                .state(State.EXTENDED_OVER_INTAKE_CLOSE)
                .onEnter(() -> dt.setDrivePower(0.9))
                .transition(() -> gamepad1.left_bumper, State.INTAKING_CLOSE, () -> {
//                    new ArmDropToGroundCommand().schedule();
//                    new WheelIntakeCommand().schedule();
                    new ClawOpenCommand().schedule();
                })
                .transition(() -> stickyG1.right_bumper, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> stickyG1.right_trigger, State.EXTENDED_OVER_INTAKE_FAR, () -> {
                    extension.teleExtendIntake(intakeExtendFar);
                })
                .transition(() -> stickyG1.a, State.SPITTING_RETRACT)

                .state(State.INTAKING_FAR)
                .onEnter(() -> {
                    dt.setDrivePower(0.45);
//                    wheel.intake();
                    claw.release();
                })
                .transition(() -> stickyG1.right_bumper || Robot.justValidSample(), State.RETRACTED, () -> {
                    if(Robot.justValidSample()) {
                        gamepad1.rumble(200);
                    }

                    new SequentialCommandGroup(
                            new ClawGrabCommand(),
//                            new WheelStopCommand(),
//                            new WristUprightForwardCommand(),
                            new WaitCommand(100),
                            new ArmRetractCommand(),
                            new BoxtubeRetractCommand()
                    ).schedule();
                })
                .transition(() -> !gamepad1.left_bumper, State.EXTENDED_OVER_INTAKE_FAR, () -> {
                    new ClawGrabCommand().schedule();
//                    new WheelStopCommand().schedule();
//                    new ArmPreIntakeCommand().schedule();
                })
                .transition(() -> stickyG1.a, State.SPITTING_FAR)
                .loop(() -> {
                    extension.teleExtendIntakeDelta(-gamepad1.right_trigger * 7.0);
                })
                .onExit(() -> {
//                    wheel.stop();
                    claw.close();
                })

                .state(State.EXTENDED_OVER_INTAKE_FAR)
                .onEnter(() -> dt.setDrivePower(0.7))
                .transition(() -> gamepad1.left_bumper, State.INTAKING_FAR, () -> {
//                    new ArmDropToGroundCommand().schedule();
//                    new WheelIntakeCommand().schedule();
                    new ClawOpenCommand().schedule();
                })
                .transition(() -> stickyG1.right_bumper, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> stickyG1.a, State.SPITTING_FAR)
                .loop(() -> {
                    extension.teleExtendIntakeDelta(-gamepad1.right_trigger * 7.0);
                })

                .state(State.SPITTING_RETRACT)
                .onEnter(() -> {
                    new SequentialCommandGroup(
//                            new ArmPreIntakeCommand(),
//                            new WaitCommand(150),
//                            new ClampReleaseCommand(),
//                            new WheelReverseCommand(),
                            new ArmRetractCommand(),
                            new WaitCommand(100),
                            new FullRetractCommand()
                    ).schedule();
                })
                .transition(() -> stickyG1.right_bumper, State.RETRACTED, () -> new FullRetractCommand().schedule())
                .transitionTimed(0.25, State.RETRACTED)

                .state(State.SPITTING_FAR)
                .onEnter(() -> {
                    extension.teleExtendIntake(intakeExtendFar);
                    new SequentialCommandGroup(
//                            new ArmPreIntakeCommand(),
//                            new ClampReleaseCommand(),
//                            new WheelReverseCommand(),
//                            new WaitCommand(100),
//                            new WheelStopCommand(),
                            new ClawGrabCommand()
                    ).schedule();
                })
                .transition(() -> stickyG1.right_bumper, State.RETRACTED, () -> new FullRetractCommand().schedule())
                .transitionTimed(0.1, State.EXTENDED_OVER_INTAKE_FAR)

                .state(State.LIFTING_BASKET_HIGH)
                .onEnter(() -> dt.setDrivePower(0.7))
                .transition(() -> stickyG1.right_bumper, State.RETRACTED, () -> {
                    new RetractFromBasketCommand().schedule();
                })
                .transition(() -> stickyG1.right_trigger, State.SCORING_BASKET_LOW, () -> {
                    new SampleBackLowCommand().schedule();
                })
                .transition(() -> extension.getDistance() > 17.0, State.SCORING_BASKET_HIGH, () -> {
                    new ArmSampleScorePositionCommand().schedule();
                })

                .state(State.SCORING_BASKET_HIGH)
                .onEnter(() -> dt.setDrivePower(0.4))
                .transition(() -> stickyG1.right_trigger, State.SCORING_BASKET_LOW, () -> {
                    new SampleBackLowCommand().schedule();
                })
                .transition(() -> stickyG1.right_bumper, State.RETRACTED, () -> {
                    new RetractFromBasketCommand().schedule();
                })
                .loop(() -> {
//                    if(gamepad1.left_bumper) {
//                        claw.release();
//                        wheel.setPower(-0.45);
//                    } else {
//                        claw.grab();
//                        wheel.stop();
//                    }
                })
                .onExit(() -> {
                    claw.grab();
//                    wheel.stop();
                })

                .state(State.SCORING_BASKET_LOW)
                .onEnter(() -> dt.setDrivePower(0.8))
                .transition(() -> stickyG1.right_trigger, State.LIFTING_BASKET_HIGH, () -> {
                    new SampleBackHighLiftCommand().schedule();
                })
                .transition(() -> stickyG1.right_bumper, State.RETRACTED, () -> {
                    new RetractFromBasketCommand().schedule();
                })
                .loop(() -> {
//                    if(gamepad1.left_bumper) {
//                        claw.release();
//                        wheel.setPower(-0.45);
//                    } else {
//                        claw.grab();
//                        wheel.stop();
//                    }
                })
//                .onExit(() -> {
//                    claw.grab();
//                    wheel.stop();
//                })

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
                .transition(() -> stickyG1.x || Robot.justValidSpecimen(), State.ABOVE_SPECIMEN_BACK, () -> {
                    new BoxtubeSplineCommand(
                            new Vector2d(20,42),
                            new Pose2d(-8.6, 30, Math.PI),
                            0,
                            0.75
                    ).schedule();
                })
                .loop(() -> {
//                    if(gamepad1.left_bumper) {
//                        claw.release();
//                        wheel.intake();
//                    } else if(gamepad1.right_bumper) {
//                        claw.release();
//                        wheel.reverse();
//                    } else {
//                        claw.grab();
//                        wheel.stop();
//                    }
                })
                .onExit(() -> {
                    claw.grab();
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

                .state(State.AUTO_SPEC_INTAKE)
                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> currentPath.isDone(), State.AUTO_SPEC_INTAKE_FAIL, () -> {
                    currentPath = new SpecimenCycleIntakeFailsafePath().start();
                })
                .transition(() -> Robot.justValidSample() && pivot.getAngle() < 0.5, State.AUTO_SPEC_DEPO_PATH, () -> {
                    currentPath = new TeleSpecimenDepoPath().start();
                })

                .state(State.AUTO_SPEC_DEPO_PATH)
                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
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
                            new ClawOpenCommand(),
                            new WaitCommand(150),
                            new PivotRetractCommand(),
                            new ExtensionRetractCommand(),
                            new WaitCommand(100),
                            new FullRetractCommand()
                    ).schedule();
                })

                .state(State.AUTO_SPEC_SCORING)
                .transitionTimed(0.2, State.AUTO_SPEC_INTAKE, () -> {
                    currentPath = new SpecimenIntakePath().start();
                })

                .state(State.AUTO_SPEC_INTAKE_FAIL)
                .transition(() -> stickyG1.a, State.RETRACTED, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> Robot.validSample(), State.AUTO_SPEC_DEPO_PATH, () -> {
                    currentPath = new TeleSpecimenDepoPath().start();
                })
                .transition(() -> currentPath.isDone() || stickyG1.b, State.AUTO_SPEC_INTAKE, () -> {
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
                            new BoxtubeHooksTopBarCommand(),
                            new WaitCommand(700),
                            new HangMotorHighBarCommand()
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
//                            new WristHorizontalCommand(),
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
//                    wrist.disable();
//                    clamp.disable();
//                    pusher.retract();
//                    pusher.disable();
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
                    turret.enable();
                    claw.enable();
//                    pusher.enable();
                })

                .state(State.HANGING)
                .onEnter(() -> hangMotor.holdPosition())
                .transition(() -> stickyG1.right_stick_button || stickyG2.right_stick_button, State.RETRACTED, () -> {
                    new HangServosRetractCommand().schedule();
                    new FullRetractCommand().schedule();
                })
                .onExit(() -> hangMotor.idle())

                .state(State.MANUAL_RESET)
                .onEnter(() -> dt.setDrivePower(0.8))
                .transition(() -> stickyG1.left_bumper, State.RETRACTED, () -> {
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

        sm.setState(State.MANUAL_RESET);
        sm.start();
    }

    @Override
    public void onStart() {
        pivot.pidTo(0);
        extension.pidTo(0);

        dt.setPoseEstimate(DriveBase.startPose);

        cvMaster.detectTag();
//        cvMaster.stopSampleStreaming();
//        cvMaster.disableSampleDetector();
    }

    @Override
    public void periodic() {
        switch (Enum.valueOf(State.class, sm.getStateString())) {
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
            case MANUAL_RESET:
            case HANGING:
                dt.drive(new Pose2d(0,0,0));
                break;
            default:
//                if(gamepad1.a)
//                    dt.driveToHeading(gamepad1.left_stick_x, -gamepad1.left_stick_y, Math.PI/4);
//                else
                dt.teleOpDrive(gamepad1);
        }

        if(gamepad1.right_stick_button) {
            dt.setHeading(-Math.toRadians(89.2), Globals.alliance);
            gamepad1.rumble(150);
        }

//        if(stickyG1.x) {
//            dt.setPoseEstimate(Globals.unMapPose(new Pose2d(6.5, -30, -Math.PI/2)));
//        }

        if(stickyG1.left_trigger) {
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
