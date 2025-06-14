package org.firstinspires.ftc.teamcode.blucru.opmode.test.kinematics;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(group = "test")
public class BoxtubeSplineDemo extends BluLinearOpMode {
    enum State {
        RETRACT,
        FOLLOWING_SEQUENCE,
        WAITING_FOR_NEXT,
        HEART_UP,
        WAITING_FOR_HEART_DOWN,
        HEART_DOWN
    }

    StateMachine sm;

    int count = 0, totalCount = 5;
    Pose2d[] poses = new Pose2d[totalCount];
    Vector2d[] vels = new Vector2d[totalCount];
    double[] wristAngles = new double[totalCount];
    double[] durations = new double[totalCount];

    @Override
    public void initialize() {
        addPivot();
        addExtension();
        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());
        addArm();
        addClaw();

        poses = new Pose2d[] {
            new Pose2d(15, 15, Math.PI/4),
            new Pose2d(15, 15, 0),
            new Pose2d(-8, 30, 3*Math.PI/4),
            new Pose2d(8, 30, 0),
            new Pose2d(8, 30, Math.PI/2)
        };

        vels = new Vector2d[] {
                new Vector2d(0,0),
                new Vector2d(0,0),
                new Vector2d(0,0),
//                new Vector2d(10,30),
                new Vector2d(0,0),
                new Vector2d(0,0)
        };

        wristAngles = new double[] {
            Math.PI/2,
            -Math.PI/2,
            Math.PI/2,
            -Math.PI/2,
            -Math.PI/2
        };

        durations = new double[] {
            0.8,
            0.8,
            0.8,
            0.8,
            0.8
        };

        sm = new StateMachineBuilder()
                .state(State.RETRACT)
                .transition(() -> stickyG1.y, State.FOLLOWING_SEQUENCE, () -> {
                    count = 0;
                    new BoxtubeSplineCommand(
                            vels[count],
                            poses[count],
                            wristAngles[count],
                            durations[count]
                    ).schedule();
                })
                .transition(() -> stickyG1.b, State.HEART_DOWN, () -> {
                    splineToHeartDown(0, -Math.PI/2);
                })
                .state(State.HEART_DOWN)
                .transition(() -> stickyG1.b, State.HEART_UP, () -> {
                    splineToHeartUp(Math.PI/2, -Math.PI/2);
                })
                .transition(() -> stickyG1.a, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })
                .state(State.HEART_UP)
                .transition(() -> robot.splineDone(), State.HEART_DOWN, () -> {
                    splineToHeartDown(0, -Math.PI/2);
                })
                .transition(() -> stickyG1.a, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })

                .state(State.WAITING_FOR_HEART_DOWN)
                .transitionTimed(500, State.HEART_DOWN, () -> {
                    splineToHeartDown(0, -Math.PI/2);
                })
                .transition(() -> stickyG1.a, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })
                .state(State.FOLLOWING_SEQUENCE)
//                .transition(() -> robot.splineDone(), State.WAITING_FOR_NEXT)
                .transition(() -> stickyG1.a, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })
                .loop(() -> {
                    if(stickyG1.y) {
                        incrementSpline();
                    }
                })
//                .state(State.WAITING_FOR_NEXT)
//                .transitionTimed(500, State.FOLLOWING_SEQUENCE, () -> {
//                    incrementSpline();
//                })
                .transition(() -> stickyG1.a, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
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
        telemetry.addData("State:", sm.getState());
        telemetry.addData("index", count);
    }

    public void splineToHeartDown(double armAngle, double wristAngle) {
        new BoxtubeSplineCommand(
                new Vector2d(-27,21),
                new Pose2d(17,8,armAngle),
                new Vector2d(45,-25),
                wristAngle,
                1
        ).schedule();
    }

    public void splineToHeartUp(double armAngle, double wristAngle) {
        new BoxtubeSplineCommand(
                new Vector2d(45,25),
                new Pose2d(17,22,armAngle),
                new Vector2d(-27,-21),
                wristAngle,
                1
        ).schedule();
    }

    public void incrementSpline() {
        count++;
        if(count >= totalCount) {
            count = 0;
        }
        new BoxtubeSplineCommand(
                vels[count],
                poses[count],
                wristAngles[count],
                durations[count]
        ).schedule();
    }
}
