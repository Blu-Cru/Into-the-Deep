package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class IntakeTest extends BluLinearOpMode {
    enum State{
        RETRACTED,
        EXTENDING_OVER_INTAKE,
        INTAKING
    }

    StateMachine sm;
    Pose2d aboveIntakePose = new Pose2d(26, 5.5, -Math.PI/2);
    Pose2d intakePose = new Pose2d(26, 2, -Math.PI/2);

    @Override
    public void initialize() {
        addDrivetrain();
        addArm();
        addClaw();
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
                            aboveIntakePose,
                            -Math.PI/2,
                            0.75
                    ).schedule();
//                    robot.setIKPose(new Pose2d(20, 6, -Math.PI/2));
                })

                .state(State.EXTENDING_OVER_INTAKE)
                .transition(() -> stickyG2.left_bumper, State.INTAKING, () -> {
                    robot.setIKPose(intakePose);
                    claw.release();
                })
                .transition(() -> stickyG2.a, State.RETRACTED, () -> {
                    new SequentialCommandGroup(
                            new BoxtubeSplineCommand(
                                    new Vector2d(-4, 10),
                                    new Pose2d(12, 4, 0),
                                    -Math.PI/2,
                                    0.4
                            ),
                            new WaitCommand(300),
                            new FullRetractCommand()
                    ).schedule();
                })
                .loop(() -> {
//                    if(gamepad2.right_bumper) {
//                        wheel.reverse();
//                        claw.release();
//                    } else {
//                        wheel.stop();
//                        claw.grab();
//                    }

                    if(stickyG2.dpad_left) {
                        turret.horizontal();
                        robot.setIKPose(aboveIntakePose);
                    }
                    if(stickyG2.dpad_down) {
                        turret.front();
                        robot.setIKPose(aboveIntakePose);
                    }
                })

                .state(State.INTAKING)
                .onEnter(() -> {
                    claw.release();
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
                    robot.setIKPose(aboveIntakePose);
                    claw.close();
                })
                .loop(() -> {
                    if(stickyG2.dpad_left) {
                        turret.horizontal();
                        robot.setIKPose(intakePose);
                    }
                    if(stickyG2.dpad_down) {
                        turret.front();
                        robot.setIKPose(intakePose);
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
