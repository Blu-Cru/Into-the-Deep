package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeExtendCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionRetractCommand;
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
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristHorizontalCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristOppositeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Specimen Test", group = "test")
public class SpecimenTest extends BluLinearOpMode {
    enum State {
        RETRACTED,
        EXTENDING_OVER_INTAKE,
        INTAKING,
        EXTENDING_TO_WALL,
        INTAKING_WALL,
        ABOVE_SPECIMEN,
        DUNKING_SPECIMEN
    }
    
    StateMachine sm;
    
    @Override
    public void initialize() {
        addDrivetrain();
        addArm();
        addWheel();
        addWrist();
        addClamp();
        addPivot();
        addExtension();

        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());
        
        dt.drivePower = 0.7;


        sm = new StateMachineBuilder()
                .state(State.RETRACTED)
                .transition(() -> -gamepad2.right_stick_y > 0.2, State.EXTENDING_OVER_INTAKE, () -> {
                    extension.setManualIntakingPower(-gamepad2.right_stick_y);
                    new ArmPreIntakeCommand().schedule();
                })
                .transition(() -> stickyG2.b, State.ABOVE_SPECIMEN, () -> {
                    new BoxtubeExtendCommand(1.4, 5).schedule();
                    new WristHorizontalCommand().schedule();
                    new ArmGlobalAngleCommand(2.5).schedule();
                })
                .transition(() -> stickyG2.x, State.EXTENDING_TO_WALL, () -> {
                    new BoxtubeExtendCommand(0.3, 5.3).schedule();
                    new WristOppositeCommand().schedule();
                    new ArmGlobalAngleCommand(0).schedule();
                })

                .state(State.EXTENDING_TO_WALL)
                .transition(() -> gamepad2.left_bumper, State.INTAKING_WALL, () -> {
                    new WheelIntakeCommand().schedule();
                    new ClampReleaseCommand().schedule();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ArmGlobalAngleCommand(1),
                            new BoxtubeExtendCommand(0.45, 5.6),
                            new WaitCommand(300),
                            new EndEffectorRetractCommand(),
                            new BoxtubeRetractCommand()
                    ).schedule();
                })

                .state(State.INTAKING_WALL)
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ClampGrabCommand(),
                            new WheelStopCommand(),
                            new ArmGlobalAngleCommand(1),
                            new BoxtubeExtendCommand(0.45, 5.6),
                            new WaitCommand(300),
                            new EndEffectorRetractCommand(),
                            new BoxtubeRetractCommand()
                    ).schedule();
                })
                .transition(() -> !gamepad2.left_bumper, State.EXTENDING_TO_WALL, () -> {
                    new ClampGrabCommand().schedule();
                    new WheelStopCommand().schedule();
                })

                .state(State.EXTENDING_OVER_INTAKE)
                .transition(() -> gamepad2.left_bumper, State.INTAKING, () -> {
                    new ArmDropToGroundCommand().schedule();
                    new WheelIntakeCommand().schedule();
                    new ClampReleaseCommand().schedule();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new ExtensionRetractCommand().schedule();
                    new EndEffectorRetractCommand().schedule();
                })
                .loop(() -> {
                    if(Math.abs(gamepad2.right_stick_y) > 0.2) extension.setManualIntakingPower(-gamepad2.right_stick_y);
                    if(gamepad2.right_bumper) {
                        wheel.reverse();
                        clamp.release();
                    } else {
                        wheel.stop();
                        clamp.grab();
                    }
                })

                .state(State.INTAKING)
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ClampGrabCommand(),
                            new WheelStopCommand(),
                            new ArmPreIntakeCommand(),
//                            new WaitCommand(300),
                            new ExtensionRetractCommand(),
                            new EndEffectorRetractCommand()
                    ).schedule();
                })
                .transition(() -> !gamepad2.left_bumper, State.EXTENDING_OVER_INTAKE, () -> {
                    new ClampGrabCommand().schedule();
                    new WheelStopCommand().schedule();
                    new ArmPreIntakeCommand().schedule();
                })
                .loop(() -> {
                    if(Math.abs(-gamepad2.right_stick_y) > 0.2) {
                        extension.setManualIntakingPower(-gamepad2.right_stick_y);
                    }
                    clamp.release();
                    wheel.intake();
                })

                .state(State.ABOVE_SPECIMEN)
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ArmGlobalAngleCommand(1.2),
                            new WaitCommand(150),
                            new BoxtubeRetractCommand(),
                            new ClampGrabCommand(),
                            new WristUprightForwardCommand(),
                            new WheelStopCommand(),
                            new ArmRetractCommand()
                    ).schedule();
                })
                .transition(() -> gamepad2.left_bumper, State.DUNKING_SPECIMEN, () -> {
                    new BoxtubeExtendCommand(1.6, 0).schedule();
                })

                .state(State.DUNKING_SPECIMEN)
                .transition(() -> !gamepad2.left_bumper, State.ABOVE_SPECIMEN, () -> {
                    new BoxtubeExtendCommand(1.4, 5).schedule();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ClampReleaseCommand(),
                            new WheelReverseCommand(),
                            new WaitCommand(300),
                            new BoxtubeRetractCommand(),
                            new WaitCommand(150),
                            new ClampGrabCommand(),
                            new WristUprightForwardCommand(),
                            new WheelStopCommand(),
                            new ArmRetractCommand()
                    ).schedule();
                })
                .build();

        sm.setState(State.RETRACTED);
        sm.start();
    }

    @Override
    public void periodic() {
        dt.teleOpDrive(gamepad1);
        if(gamepad1.right_stick_button) dt.resetHeading(Math.PI/2);
        sm.update();
    }

    @Override
    public void telemetry() {
        telemetry.addData("state", sm.getState());
    }
}
