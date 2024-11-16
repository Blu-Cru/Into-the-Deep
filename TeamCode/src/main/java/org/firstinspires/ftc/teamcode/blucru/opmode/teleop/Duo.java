package org.firstinspires.ftc.teamcode.blucru.opmode.teleop;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.BackHighBasketCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.BackLowBasketCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FrontLowBasketCommand;
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
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightBackwardCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "Main TeleOp", group = "1")
public class Duo extends BluLinearOpMode {
    enum State {
        RETRACTED,
        EXTENDING_OVER_INTAKE,
        INTAKING,
        SCORING_BASKET,
        RETRACTING_FROM_SCORING,
        RETRACTING_FROM_INTAKE
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
        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());

        dt.drivePower = 0.8;

        sm = new StateMachineBuilder()
                .state(State.RETRACTED)
                .onEnter(() -> dt.drivePower = 0.9)
                .transition(() -> -gamepad2.right_stick_y > 0.2, State.EXTENDING_OVER_INTAKE, () -> {
                    extension.setManualIntakingPower(-gamepad2.right_stick_y);
                    new ArmPreIntakeCommand().schedule();
                })
                .transition(() -> stickyG2.b && !gamepad2.dpad_left, State.SCORING_BASKET, () -> {
                    new BackLowBasketCommand().schedule();
                })
                .transition(() -> stickyG2.y && !gamepad2.dpad_left, State.SCORING_BASKET, () -> {
                    new BackHighBasketCommand().schedule();
                })
                .transition(() -> stickyG2.b && gamepad2.dpad_left, State.SCORING_BASKET, () -> {
                    new FrontLowBasketCommand().schedule();
                })

                .state(State.EXTENDING_OVER_INTAKE)
                .onEnter(() -> dt.drivePower = 0.75)
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
                .onEnter(() -> dt.drivePower = 0.75)
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

                .state(State.SCORING_BASKET)
                .onEnter(() -> dt.drivePower = 0.45)
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new ArmGlobalAngleCommand(1.5),
                            new WaitCommand(150),
                            new BoxtubeRetractCommand(),
                            new ClampGrabCommand(),
                            new WristUprightForwardCommand(),
                            new WheelStopCommand(),
                            new WaitCommand(100),
                            new ArmRetractCommand()
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

                    if(stickyG2.y) {
                        new BoxtubeExtendCommand(1.6, 22).schedule();
                        new WristUprightForwardCommand().schedule();
                        new ArmGlobalAngleCommand(2.2).schedule();
                    }
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
