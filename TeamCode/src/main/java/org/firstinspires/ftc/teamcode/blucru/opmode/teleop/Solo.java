package org.firstinspires.ftc.teamcode.blucru.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.spin_wrist.SpinWristGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretCenterCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.turret.TurretMotionProfileCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.up_down_wrist.UpDownWristAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.GrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.SpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.sample.SampleBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.sample.SampleBackLowCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenFrontClipUnderneathCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenFrontFlatCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackClipCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackFlatCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.specimen.SpecimenIntakeBackFlatSpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenCycleIntakeFailsafePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpitIntakeSpecPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.tele.TeleSpecimenDepoPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.boxtube.kinematics.BoxtubeKinematics;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.DriveBase;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector.Arm;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.end_effector.Claw;
import org.firstinspires.ftc.teamcode.blucru.common.util.SampleOrientation;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(group = "2")
public class Solo extends BluLinearOpMode {
    public static double INTAKE_POS_X = 30.0, INTAKE_POS_Y_DELTA = 4.0;
    enum State{
        HOME,
        PREINTAKE,
        INTAKING_SPEC,
        SCORING_SPEC,
        SCORING_BASKET,

        AUTO_SPEC_INTAKE,
        AUTO_SPEC_INTAKE_FAILSAFE,
        AUTO_SPEC_DRIVE_TO_CHAMBER,

        GRABBED_GROUND,
        SENSING_GROUND,
        GRABBED_SPEC,
        SENSING_SPEC,

        MANUAL_RESET
    }

    StateMachine sm;
    SampleOrientation orientation = SampleOrientation.VERTICAL;
    boolean grabByClip;
    Path currentPath;

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
        addCVMaster();

        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());

        dt.setDrivePower(0.7);
        grabByClip = false;

        sm = new StateMachineBuilder()
                .state(State.HOME)
                .onEnter(() -> dt.setDrivePower(1.0))
                .transition(() -> stickyG1.share, State.MANUAL_RESET, () -> {
                    gamepad1.rumble(350);
                })
                .transition(() -> (stickyG1.left_bumper) && pivot.getAngle() < 0.4, State.PREINTAKE, () -> {
                    double[] joints = BoxtubeKinematics.getExtensionTurretPose(new Vector2d(INTAKE_POS_X, 0));

                    orientation = SampleOrientation.VERTICAL;
                    new SequentialCommandGroup(
                            new TurretCenterCommand(),
                            new PreIntakeCommand(),
                            new WaitCommand(320),
                            new SpinWristGlobalAngleCommand(SampleOrientation.VERTICAL),
                            new ExtensionCommand(joints[0]),
                            new WaitCommand(230),
                            new ClawOpenCommand()
                    ).schedule();
                })
                .transition(() -> stickyG1.y && extension.getDistance() < 2.0, State.SCORING_BASKET, () -> {
                    new SampleBackHighCommand().schedule();
                })
                .transition(() -> stickyG1.x && extension.getDistance() < 2.0, State.INTAKING_SPEC, () -> {
                    if (grabByClip) {
                        new SpecimenIntakeBackClipCommand().schedule();
                    } else {
                        if (Robot.validSample()) {
                            new SpecimenIntakeBackFlatSpitCommand().schedule();
                        } else {
                            new SpecimenIntakeBackFlatCommand().schedule();
                        }
                    }
                })
                .transition(() -> stickyG1.b, State.AUTO_SPEC_INTAKE, () -> {
                    if (dt.updateAprilTags())
                        gamepad1.rumble(250);

                    if (cactus.validSample())
                        currentPath = new SpitIntakeSpecPath().build().start();
                    else
                        currentPath = new SpecimenIntakePath().build().start();
                })
                .loop(() -> {
                    if(stickyG1.right_bumper) new SpitCommand().schedule();
                })

                .state(State.PREINTAKE)
                .onEnter(() -> {
                    dt.setDrivePower(0.30);
                })
                .transition(() -> stickyG1.a, State.HOME, () -> {
                    new FullRetractCommand().schedule();
                })
                .transition(() -> stickyG1.left_bumper, State.GRABBED_GROUND, () -> {
                    new GrabCommand().schedule();
                })

                .loop(() -> {
                    if (stickyG1.left_trigger) {
                        double[] joints = BoxtubeKinematics.getExtensionTurretPose(new Vector2d(INTAKE_POS_X, INTAKE_POS_Y_DELTA));
                        new SequentialCommandGroup(
                                new TurretMotionProfileCommand(joints[1]),
                                new ExtensionCommand(joints[0])
                        ).schedule();
                    }
                    if (stickyG1.right_trigger) {
                        double[] joints = BoxtubeKinematics.getExtensionTurretPose(new Vector2d(INTAKE_POS_X, -INTAKE_POS_Y_DELTA));
                        new SequentialCommandGroup(
                                new TurretMotionProfileCommand(joints[1]),
                                new ExtensionCommand(joints[0])
                        ).schedule();
                    }
                    if (stickyG1.left_trigger_released || stickyG1.right_trigger_released) {
                        double[] joints = BoxtubeKinematics.getExtensionTurretPose(new Vector2d(INTAKE_POS_X, 0));
                        new SequentialCommandGroup(
                                new TurretMotionProfileCommand(joints[1]),
                                new ExtensionCommand(joints[0])
                        ).schedule();
                    }

                    if (stickyG1.right_bumper)
                        orientation = spinWrist.setGlobalAngle(orientation.next());
                    if (stickyG1.dpad_down)
                        orientation = spinWrist.setGlobalAngle(orientation.prev());
                })

                .state(State.GRABBED_GROUND)
                .transitionTimed(0.63, State.SENSING_GROUND)
                .transition(() -> stickyG1.a, State.HOME, () -> new FullRetractCommand().schedule())
                .state(State.SENSING_GROUND)
                .onEnter(() -> dt.setDrivePower(0.6))
                .transition(() -> stickyG1.a, State.HOME, () -> new FullRetractCommand().schedule())
                .transition(() -> cactus.validSample(), State.HOME, () -> {
                    new FullRetractCommand().schedule();
                    gamepad1.rumble(80);
                })
                .transition(() -> cactus.empty(), State.PREINTAKE, () -> {
                    new SequentialCommandGroup(
                            new SpinWristGlobalAngleCommand(orientation),
                            new PreIntakeCommand(),
                            new ClawOpenCommand()
                    ).schedule();
                })
                .transitionTimed(0.15, State.PREINTAKE, () -> {
                    new SequentialCommandGroup(
                            new SpinWristGlobalAngleCommand(orientation),
                            new PreIntakeCommand(),
                            new ClawOpenCommand()
                    ).schedule();
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
                .loop(() -> {
                    if(stickyG1.y) {
                        new SampleBackHighCommand().schedule();
                    }
                    if(stickyG1.b) {
                        new SampleBackLowCommand().schedule();
                    }
                })

                .state(State.INTAKING_SPEC)
                .onEnter(() -> dt.setDrivePower(0.7))
                .transition(() -> stickyG1.a, State.HOME, () -> new FullRetractCommand().schedule())
                .transition(() -> stickyG1.b, State.AUTO_SPEC_INTAKE, () -> {
                    currentPath = new SpecimenIntakePath().start();
                })
                .transition(() -> (stickyG1.left_bumper || cactus.justValidSample()) && pivot.getAngle() > 1.3, State.GRABBED_SPEC, () -> {
                    if(cactus.justValidSample())
                        gamepad1.rumble(80);

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
                .transition(() -> stickyG1.a, State.HOME, () -> new FullRetractCommand().schedule())
                .transitionTimed(0.3, State.SENSING_SPEC)
                .state(State.SENSING_SPEC)
                .transition(() -> stickyG1.a, State.HOME, () -> new FullRetractCommand().schedule())
                .transitionTimed(0.1, State.INTAKING_SPEC, () -> {
                    if(grabByClip) {
                        new SpecimenIntakeBackClipCommand().schedule();
                    } else {
                        new SequentialCommandGroup(
                                new ClawOpenCommand(),
                                new SpecimenIntakeBackFlatCommand()
                        ).schedule();
                    }
                })
                .transition(() -> cactus.validSample() || gamepad1.left_bumper, State.SCORING_SPEC, () -> {
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
                .transition(() -> stickyG1.left_bumper, State.AUTO_SPEC_INTAKE, () -> {
                    currentPath = new SpecimenIntakePath().start();
//                    new SequentialCommandGroup(
//                            new ClawOpenCommand(),
//                            new ArmCommand(0.2),
//                            new WaitCommand(130),
//                            new ConditionalCommand(
//                                    new SpecimenIntakeBackClipCommand(),
//                                    new SpecimenIntakeBackFlatCommand(),
//                                    () -> grabByClip
//                            )
//                    ).schedule();
                })
                .transition(() -> stickyG1.a, State.HOME, () -> {
                    new SequentialCommandGroup(
                            new ClawOpenCommand(),
                            new ArmCommand(0.2),
                            new WaitCommand(130),
                            new FullRetractCommand()
                    ).schedule();
                })

                .state(State.AUTO_SPEC_INTAKE)
                .transition(() -> stickyG1.a, State.HOME, () -> new FullRetractCommand().schedule())
                .transition(() -> currentPath.isDone() && cactus.validSample(), State.AUTO_SPEC_DRIVE_TO_CHAMBER, () -> {
                    currentPath = new TeleSpecimenDepoPath().build().start();
                })
                .transition(() -> currentPath.isDone() && !cactus.validSample(), State.AUTO_SPEC_INTAKE_FAILSAFE, () -> {
                    currentPath = new SpecimenCycleIntakeFailsafePath().build().start();
                })
                .transition(() -> stickyG1.right_bumper, State.INTAKING_SPEC, () -> {
                    gamepad1.rumble(150);
                    new SpecimenIntakeBackFlatCommand().schedule();
                })

                .state(State.AUTO_SPEC_INTAKE_FAILSAFE)
                .transition(() -> stickyG1.a, State.HOME, () -> new FullRetractCommand().schedule())
                .transition(() -> stickyG1.b, State.INTAKING_SPEC, () -> {
                    new SpecimenIntakeBackFlatCommand().schedule();
                })
                .transition(() -> currentPath.isDone(), State.AUTO_SPEC_INTAKE, () -> {
                    currentPath = new SpecimenIntakePath().build().start();
                })

                .state(State.AUTO_SPEC_DRIVE_TO_CHAMBER)
                .transition(() -> stickyG1.a, State.HOME, () -> new FullRetractCommand().schedule())
                .transition(() -> isDriving() || (currentPath.isDone() && cactus.validSample()), State.SCORING_SPEC)
                .transition(() -> currentPath.isDone() && !cactus.validSample(), State.AUTO_SPEC_INTAKE, () -> {
                    currentPath = new SpecimenIntakePath().build().start();
                })

                .state(State.MANUAL_RESET)
                .transition(() -> stickyG1.left_bumper, State.HOME, () -> {
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
    }

    @Override
    public void periodic() {
        switch (Enum.valueOf(State.class, sm.getStateString())) {
            case MANUAL_RESET:
                dt.drive(new Pose2d(0, 0, 0));
                break;
            case AUTO_SPEC_INTAKE:
            case AUTO_SPEC_INTAKE_FAILSAFE:
            case AUTO_SPEC_DRIVE_TO_CHAMBER:
                currentPath.run();
                break;
            default:
                dt.teleOpDrive(gamepad1);
                break;
        }

        if(gamepad1.right_stick_button) {
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

    boolean isDriving() {
        return Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1;
    }
}
