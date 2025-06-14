package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmGlobalAngleCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.end_effector.claw.ClawOpenCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Disabled
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
        addClaw();
        addPivot();
        addExtension();

        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());
        
        dt.setDrivePower(0.7);

        sm = new StateMachineBuilder()
                .state(State.RETRACTED)
                .transition(() -> -gamepad2.right_stick_y > 0.2, State.EXTENDING_OVER_INTAKE, () -> {
                    extension.manualExtendOverIntake(-gamepad2.right_stick_y);
                })
                .transition(() -> stickyG2.b, State.ABOVE_SPECIMEN, () -> {
                    new BoxtubeCommand(1.4, 5).schedule();
//                    new WristOppositeCommand().schedule();
                    new ArmGlobalAngleCommand(2.5).schedule();
                })
                .transition(() -> stickyG2.x, State.EXTENDING_TO_WALL, () -> {
                    new BoxtubeCommand(0.3, 5.3).schedule();
//                    new WristHorizontalCommand().schedule();
                    new ArmGlobalAngleCommand(0).schedule();
                })
                .transition(() -> stickyG2.y, State.EXTENDING_TO_WALL, () -> {
                    new BoxtubeCommand(0.43, 0).schedule();
//                    new WristOppositeCommand().schedule();
                    new ArmGlobalAngleCommand(0).schedule();
                })

                .state(State.EXTENDING_TO_WALL)
                .transition(() -> gamepad2.left_bumper, State.INTAKING_WALL, () -> {
                    new ClawOpenCommand().schedule();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ArmGlobalAngleCommand(1),
                            new PivotCommand(0.5),
                            new WaitCommand(300),
                            new EndEffectorRetractCommand(),
                            new BoxtubeRetractCommand()
                    ).schedule();
                })

                .state(State.INTAKING_WALL)
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ClawGrabCommand(),
                            new ArmGlobalAngleCommand(1),
                            new BoxtubeCommand(0.45, 5.6),
                            new WaitCommand(300),
                            new EndEffectorRetractCommand(),
                            new BoxtubeRetractCommand()
                    ).schedule();
                })
                .transition(() -> !gamepad2.left_bumper, State.EXTENDING_TO_WALL, () -> {
                    new ClawGrabCommand().schedule();
                })

                .state(State.EXTENDING_OVER_INTAKE)
                .transition(() -> gamepad2.left_bumper, State.INTAKING, () -> {
                    new ClawOpenCommand().schedule();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new ExtensionRetractCommand().schedule();
                    new EndEffectorRetractCommand().schedule();
                })
                .loop(() -> {
                    if(Math.abs(gamepad2.right_stick_y) > 0.2) extension.manualExtendOverIntake(-gamepad2.right_stick_y);
//                    if(gamepad2.right_bumper) {
//                        wheel.reverse();
//                        claw.release();
//                    } else {
//                        wheel.stop();
//                        claw.grab();
//                    }
                })

                .state(State.INTAKING)
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ClawGrabCommand(),
//                            new WheelStopCommand(),
//                            new ArmPreIntakeCommand(),
//                            new WaitCommand(300),
                            new ExtensionRetractCommand(),
                            new EndEffectorRetractCommand()
                    ).schedule();
                })
                .transition(() -> !gamepad2.left_bumper, State.EXTENDING_OVER_INTAKE, () -> {
                    new ClawGrabCommand().schedule();
//                    new WheelStopCommand().schedule();
//                    new ArmPreIntakeCommand().schedule();
                })
                .loop(() -> {
                    if(Math.abs(-gamepad2.right_stick_y) > 0.2) {
                        extension.manualExtendOverIntake(-gamepad2.right_stick_y);
                    }
                    claw.release();
                })

                .state(State.ABOVE_SPECIMEN)
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ArmGlobalAngleCommand(1.2),
                            new WaitCommand(150),
                            new BoxtubeRetractCommand(),
                            new ClawGrabCommand(),
//                            new WristUprightForwardCommand(),
                            new ArmRetractCommand()
                    ).schedule();
                })
                .transition(() -> gamepad2.left_bumper, State.DUNKING_SPECIMEN, () -> {
                    new BoxtubeCommand(1.6, 0).schedule();
                })

                .state(State.DUNKING_SPECIMEN)
                .transition(() -> !gamepad2.left_bumper, State.ABOVE_SPECIMEN, () -> {
                    new BoxtubeCommand(1.4, 5).schedule();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
//                            new ClampReleaseCommand(),
//                            new WheelReverseCommand(),
//                            new WaitCommand(300),
//                            new BoxtubeRetractCommand(),
//                            new WaitCommand(150),
//                            new ClampGrabCommand(),
//                            new WristUprightForwardCommand(),
//                            new WheelStopCommand(),
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
        if(gamepad1.right_stick_button) dt.setHeading(Math.PI/2);
        sm.update();
    }

    @Override
    public void telemetry() {
        telemetry.addData("state", sm.getState());
    }
}
