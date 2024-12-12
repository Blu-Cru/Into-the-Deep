package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wrist.WristUprightForwardCommand;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name = "Boxtube Kinematics Test", group = "test")
public class BoxtubeKinematicsTest extends BluLinearOpMode {
    enum State {
        RETRACT,
        IVK
    }
    public static double x = 10, y = 27, angle = 1.4;
    Pose2d targetPose = new Pose2d(x, y, angle);
    StateMachine sm;
    double controlledX, controlledY, controlledAngle;

    @Override
    public void initialize() {
        addArm();
        addWrist();
        addClamp();
        addPivot();
        addExtension();
        pivot.useExtension(extension.getMotor());
        extension.usePivot(pivot.getMotor());

        sm = new StateMachineBuilder()
                .state(State.RETRACT)
                .onEnter(() -> {
                    new BoxtubeRetractCommand().schedule();
                    new ArmRetractCommand().schedule();
                    new WristUprightForwardCommand().schedule();
                })
                .transition(() -> stickyG1.b, State.IVK, () -> {
                    robot.setIKPose(targetPose);
                })
                .state(State.IVK)
                .loop(() -> {
//                    wrist.setAngle(-Math.PI/2 + Math.PI * -gamepad1.right_stick_x);
                    robot.setIKPose(targetPose);
                })
                .transition(() -> stickyG1.a, State.RETRACT)
                .build();
    }

    @Override
    public void onStart() {
        sm.setState(State.RETRACT);
        sm.start();
    }

    @Override
    public void periodic() {
        controlledX = x + 5.0 * gamepad1.left_stick_x;
        controlledY = y + 5.0 * -gamepad1.left_stick_y;
        controlledAngle = angle - 0.85 * -gamepad1.right_stick_y;

        targetPose = new Pose2d(controlledX, controlledY, controlledAngle);
        sm.update();
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", sm.getState());
        telemetry.addData("X", controlledX);
        telemetry.addData("Y", controlledY);
        telemetry.addData("Angle", controlledAngle);
    }
}
