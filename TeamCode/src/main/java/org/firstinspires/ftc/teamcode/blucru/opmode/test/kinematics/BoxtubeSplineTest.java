package org.firstinspires.ftc.teamcode.blucru.opmode.test.kinematics;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name = "Boxtube Spline Test",group = "test")
public class BoxtubeSplineTest extends BluLinearOpMode {
    public static double xVel = 0, yVel = 0,
            x = 0, y = 0, blockAngle = 0,
            wristAngle = 0,
            duration = 1;   // seconds


    enum State {
        RETRACT,
        FOLLOWING_SPLINE
    }

    StateMachine sm;

    @Override
    public void initialize() {
        addPivot();
        addExtension();
        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());
        addWrist();
        addArm();
        addClamp();

        sm = new StateMachineBuilder()
                .state(State.RETRACT)
                .transition(() -> stickyG1.y, State.FOLLOWING_SPLINE, () -> {
                    new BoxtubeSplineCommand(
                            new Vector2d(x, y),
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
                .transition(() -> stickyG1.a, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })
                .loop(() -> {
                    if(stickyG1.b) {
                        new BoxtubeSplineCommand(
                                new Vector2d(0,0),
                                new Pose2d(x, y, blockAngle),
                                wristAngle,
                                duration
                        ).schedule();
                    }

                    if(stickyG1.y) {
                        new BoxtubeSplineCommand(
                                new Vector2d(x, y),
                                new Pose2d(x, y, blockAngle),
                                wristAngle,
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
        telemetry.addData("State", sm.getState());
    }
}
