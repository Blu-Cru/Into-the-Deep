package org.firstinspires.ftc.teamcode.blucru.opmode.test;

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
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmDropToGroundCommand;
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

@TeleOp(group = "test")
public class IntakeTest extends BluLinearOpMode {
    enum State{
        RETRACTED,
        EXTENDING_OVER_INTAKE,
        INTAKING
    }

    StateMachine sm;

    @Override
    public void initialize() {
        addDrivetrain();
        addArm();
        addWheel();
        addClamp();
        addWrist();
        addPivot();
        addExtension();

        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());

        dt.setDrivePower(0.7);

        sm = new StateMachineBuilder()
                .state(State.RETRACTED)
                .transition(() -> stickyG2.left_bumper, State.EXTENDING_OVER_INTAKE, () -> {
                    new BoxtubeSplineCommand(
                            new Pose2d(20, 5.5, -Math.PI/2),
                            -Math.PI/2,
                            0.85
                    ).schedule();
//                    robot.setIKPose(new Pose2d(20, 6, -Math.PI/2));
                })

                .state(State.EXTENDING_OVER_INTAKE)
                .transition(() -> stickyG2.left_bumper, State.INTAKING, () -> {
                    robot.setIKPose(new Pose2d(20, 2, -Math.PI/2));
                    clamp.release();
                    wheel.intake();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new BoxtubeSplineCommand(
                                    new Vector2d(-4, 10),
                                    new Pose2d(12, 4.5, 0),
                                    -Math.PI/2,
                                    0.4
                            ),
                            new WaitCommand(300),
                            new FullRetractCommand()
                    ).schedule();
                })
                .loop(() -> {
                    if(gamepad2.right_bumper) {
                        wheel.reverse();
                        clamp.release();
                    } else {
                        wheel.stop();
                        clamp.grab();
                    }

                    if(stickyG2.dpad_left) {
                        wrist.horizontal();
                        robot.setIKPose(new Pose2d(20, 5.5, -Math.PI/2));
                    }
                    if(stickyG2.dpad_down) {
                        wrist.front();
                        robot.setIKPose(new Pose2d(20, 5.5, -Math.PI/2));
                    }
                })

                .state(State.INTAKING)
                .onEnter(() -> {
                    wheel.intake();
                    clamp.release();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new BoxtubeSplineCommand(
                                    new Vector2d(-4, 18),
                                    new Pose2d(12, 5.5, 0),
                                    -Math.PI/2,
                                    0.35
                            ),
                            new WaitCommand(300),
                            new FullRetractCommand()
                    ).schedule();
                })
                .transition(() -> !gamepad2.left_bumper, State.EXTENDING_OVER_INTAKE, () -> {
                    robot.setIKPose(new Pose2d(20, 5.5, -Math.PI/2));
                    clamp.close();
                    wheel.stop();
                })
                .loop(() -> {
                    if(stickyG2.dpad_left) {
                        wrist.horizontal();
                        robot.setIKPose(new Pose2d(20, 2, -Math.PI/2));
                    }
                    if(stickyG2.dpad_down) {
                        wrist.front();
                        robot.setIKPose(new Pose2d(20, 2, -Math.PI/2));
                    }
                })
                .build();

        sm.setState(State.RETRACTED);
        sm.start();
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
        telemetry.addData("State", sm.getState());
    }
}
