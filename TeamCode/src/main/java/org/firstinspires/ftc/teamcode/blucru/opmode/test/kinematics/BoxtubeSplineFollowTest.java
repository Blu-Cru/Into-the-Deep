package org.firstinspires.ftc.teamcode.blucru.opmode.test.kinematics;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(group = "test")
public class BoxtubeSplineFollowTest extends BluLinearOpMode {
    public static double xVel = 26, yVel = 43,
            x = -8.5, y = 28.5, blockAngle = Math.PI,
            wristAngle = 0,
            duration = 1.5;   // seconds


    enum State {
        RETRACT,
        FOLLOWING_SPLINE
    }

    StateMachine sm;
    Pose2d retractPose;

    @Override
    public void initialize() {
        addPivot();
        addExtension();
        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());
        addWrist();
        addArm();
        addClamp();

        enableFTCDashboard();

        retractPose = robot.getBoxtubePose();

        sm = new StateMachineBuilder()
                .state(State.RETRACT)
                .transition(() -> stickyG1.y, State.FOLLOWING_SPLINE, () -> {
                    new BoxtubeSplineCommand(
                            new Vector2d(xVel, yVel),
                            new Pose2d(x, y, blockAngle),
                            wristAngle,
                            duration
                    ).schedule();
                })
                .transition(() -> stickyG1.b, State.FOLLOWING_SPLINE, () -> {
                    new BoxtubeSplineCommand(
                            new Vector2d(0,0),
                            new Pose2d(x, y, blockAngle),
                            wristAngle,
                            duration
                    ).schedule();
                })
                .state(State.FOLLOWING_SPLINE)
                .transition(() -> stickyG1.dpad_down, State.RETRACT, () -> {
                    new BoxtubeRetractCommand().schedule();
                    new WristUprightForwardCommand().schedule();
                    new ArmRetractCommand().schedule();
                })
                .loop(() -> {
                    if(stickyG1.b) {
                        new BoxtubeSplineCommand(
                                new Vector2d(0,0),
                                new Pose2d(21, 9, 0),
                                -Math.PI/2,
                                1.0
                        ).schedule();
                    }

                    if(stickyG1.y) {
                        new BoxtubeSplineCommand(
                                new Vector2d(xVel, yVel),
                                new Pose2d(-10, 22, 2.2),
                                0,
                                1.5
                        ).schedule();
                    }

                    if(stickyG1.dpad_up) {
                        new BoxtubeSplineCommand(
                                new Vector2d(xVel, yVel),
                                new Pose2d(x, y, blockAngle),
                                wristAngle,
                                duration
                        ).schedule();
                    }

                    if(stickyG1.a) {
                        new BoxtubeSplineCommand(
                                new Pose2d(8, 11, 1.6),
                                -Math.PI/2,
                                duration
                        ).schedule();
                    }
                })
                .build();
    }

    @Override
    public void onStart() {
        sm.setState(State.RETRACT);
        sm.start();
    }

    @Override
    public void periodic() {
        sm.update();
    }

    @Override
    public void telemetry() {
        telemetry.addData("Retract Pose", retractPose);
        telemetry.addData("State", sm.getState());
        telemetry.addData("Arm get angle", arm.getAngle());
        telemetry.addData("Wrist get angle", wrist.getAngle());
    }
}
