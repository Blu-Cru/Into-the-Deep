package org.firstinspires.ftc.teamcode.blucru.opmode.teleop;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleBackHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleBackLowCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleFrontHighCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.sample.SampleFrontLowCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
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
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Main", group = "1")
public class Main extends BluLinearOpMode {
    enum State {
        RETRACTED,
        EXTENDING_OVER_INTAKE,
        INTAKING_GROUND,
        SCORING_BASKET,
        EXTENDING_TO_SPECIMEN,
        INTAKING_SPECIMEN,
        ABOVE_SPECIMEN_FRONT,
        ABOVE_SPECIMEN_BACK,
        DUNKING_SPECIMEN_FRONT,
        DUNKING_SPECIMEN_BACK,
        RETRACTING_FROM_SCORING,
        RETRACTING_FROM_INTAKE,
        MANUAL_RESET
    }

    StateMachine sm;

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
        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());

        dt.setPoseEstimate(Globals.startPose);

        sm = new StateMachineBuilder()
                .state(State.RETRACTED)
                .onEnter(() -> dt.setDrivePower(0.75))
                .transition(() -> stickyG2.share, State.MANUAL_RESET, () -> {
                    gamepad1.rumble(350);
                    gamepad2.rumble(350);
                })
                .transition(() -> -gamepad2.right_stick_y > 0.2, State.EXTENDING_OVER_INTAKE, () -> {
                    extension.extendOverIntake(-gamepad2.right_stick_y);
                    new ArmPreIntakeCommand().schedule();
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

                // SPECIMEN
                .transition(() -> stickyG2.dpad_down, State.EXTENDING_TO_SPECIMEN, () -> {
                    new BoxtubeExtendCommand(0.42, 0).schedule();
                    new WristOppositeCommand().schedule();
                    new ArmGlobalAngleCommand(0).schedule();
                })
                .transition(() -> stickyG2.x && !gamepad2.dpad_left, State.ABOVE_SPECIMEN_BACK, () ->
                        new SpecimenBackCommand().schedule())
                .transition(() -> stickyG2.x && gamepad2.dpad_left, State.ABOVE_SPECIMEN_FRONT, () ->
                        new SpecimenFrontCommand().schedule())
                .loop(() -> {
                    if(stickyG2.right_bumper) {
                        new SequentialCommandGroup(
                                new ArmPreIntakeCommand(),
                                new WaitCommand(150),
                                new ClampReleaseCommand(),
                                new WheelReverseCommand(),
                                new WaitCommand(300),
                                new EndEffectorRetractCommand()
                        ).schedule();
                    }
                })

                .state(State.EXTENDING_OVER_INTAKE)
                .onEnter(() -> dt.setDrivePower(0.7))
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
                    if(Math.abs(gamepad2.right_stick_y) > 0.2) extension.extendOverIntake(-gamepad2.right_stick_y);
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
                    if(Math.abs(-gamepad2.right_stick_y) > 0.2) {
                        extension.extendOverIntake(-gamepad2.right_stick_y);
                    }
                })

                .onExit(() -> {
                    wheel.stop();
                    clamp.close();
                })

                .state(State.EXTENDING_TO_SPECIMEN)
                .onEnter(() -> dt.setDrivePower(0.4))
                .transition(() -> gamepad2.left_bumper, State.INTAKING_SPECIMEN, () -> {
                    new WheelIntakeCommand().schedule();
                    new ClampReleaseCommand().schedule();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ArmGlobalAngleCommand(1.2),
                            new PivotCommand(1),
                            new WaitCommand(300),
                            new BoxtubeRetractCommand(),
                            new EndEffectorRetractCommand()
                    ).schedule();
                })
                .transition(() -> stickyG2.x && !gamepad2.dpad_left, State.ABOVE_SPECIMEN_BACK, () -> {
                    new SequentialCommandGroup(
                            new ArmGlobalAngleCommand(2),
                            new PivotCommand(0.9),
                            new WaitCommand(600),
                            new SpecimenBackCommand()
                    ).schedule();
                })
                .transition(() -> stickyG2.x && gamepad2.dpad_left, State.ABOVE_SPECIMEN_FRONT, () -> {
                    new SequentialCommandGroup(
                            new ArmGlobalAngleCommand(1.2),
                            new PivotCommand(1),
                            new WaitCommand(300),
                            new BoxtubeExtendCommand(1.4, 5),
                            new WristOppositeCommand(),
                            new ArmGlobalAngleCommand(0.64)
                    ).schedule();
                })

                .state(State.INTAKING_SPECIMEN)
                .onEnter(() -> dt.setDrivePower(0.35))
                .transition(() -> !gamepad2.left_bumper, State.EXTENDING_TO_SPECIMEN, () -> {
                    new ClampGrabCommand().schedule();
                    new WheelStopCommand().schedule();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ClampGrabCommand(),
                            new WheelStopCommand(),
                            new WaitCommand(150),
                            new ArmGlobalAngleCommand(1.2),
                            new PivotCommand(1),
                            new WaitCommand(250),
                            new BoxtubeRetractCommand(),
                            new EndEffectorRetractCommand()
                    ).schedule();
                })
                .transition(() -> stickyG2.x && !gamepad2.dpad_left, State.ABOVE_SPECIMEN_BACK, () -> {
                    new SequentialCommandGroup(
                            new ClampGrabCommand(),
                            new WheelStopCommand(),
                            new WaitCommand(150),
                            new ArmGlobalAngleCommand(2),
                            new PivotCommand(1),
                            new WaitCommand(600),
                            new SpecimenBackCommand()
                    ).schedule();
                })
                .transition(() -> stickyG2.x && gamepad2.dpad_left, State.ABOVE_SPECIMEN_FRONT, () -> {
                    new SequentialCommandGroup(
                            new ClampGrabCommand(),
                            new WheelStopCommand(),
                            new WaitCommand(150),
                            new ArmGlobalAngleCommand(1.2),
                            new PivotCommand(1),
                            new WaitCommand(300),
                            new SpecimenFrontCommand()
                    ).schedule();
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
                .onEnter(() -> dt.setDrivePower(0.5))
                .transition(() -> gamepad2.left_bumper, State.DUNKING_SPECIMEN_BACK,
                        () -> new SpecimenBackDunkCommand().schedule())

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
                .onEnter(() -> dt.setDrivePower(0.4))
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
                            new WaitCommand(250),
                            new BoxtubeRetractCommand(),
                            new EndEffectorRetractCommand()
                    ).schedule();
                })
                .loop(() -> {
                    if(gamepad2.left_bumper) {
                        clamp.release();
                        wheel.reverse();
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

                .state(State.MANUAL_RESET)
                .transition(() -> stickyG2.share, State.RETRACTED, () -> {
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

        sm.setState(State.RETRACTED);
        sm.start();
    }

    @Override
    public void onStart() {
        pivot.pidTo(0);
        extension.pidTo(0);
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
        telemetry.addData("state", sm.getState());
    }
}
