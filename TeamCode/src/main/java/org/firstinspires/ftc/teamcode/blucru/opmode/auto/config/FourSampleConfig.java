package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeCenterPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeLeftPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeRightPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleLiftingPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleParkPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleScoreHighPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class FourSampleConfig extends AutoConfig {
    Path liftingPath, scorePath,
        rightIntakePath, centerIntakePath, leftIntakePath,
        rightFailsafePath, centerFailsafePath, leftFailsafePath,
        parkPath;

    enum State{
        LIFTING,
        DEPOSITING,
        RIGHT_INTAKE,
        CENTER_INTAKE,
        LEFT_INTAKE,
        RIGHT_FAILSAFE,
        CENTER_FAILSAFE,
        LEFT_FAILSAFE,
        PARKING,
        DONE
    }

    public State[] statesAfterDeposit;
    public Path[] pathsAfterDeposit;

    int scoreCount;

    public FourSampleConfig() {
        runtime = Globals.runtime;

        statesAfterDeposit = new State[3];
        statesAfterDeposit[0] = State.RIGHT_INTAKE;
        statesAfterDeposit[1] = State.CENTER_INTAKE;
        statesAfterDeposit[2] = State.LEFT_INTAKE;
        statesAfterDeposit[3] = State.PARKING;

        pathsAfterDeposit = new Path[3];
        pathsAfterDeposit[0] = rightIntakePath;
        pathsAfterDeposit[1] = centerIntakePath;
        pathsAfterDeposit[2] = leftIntakePath;
        pathsAfterDeposit[3] = parkPath;

        scoreCount = 0;

        sm = new StateMachineBuilder()
                .state(State.LIFTING)
                .transition(() -> Robot.getInstance().extension.getPIDError() < 2 && currentPath.isDone(),
                        State.DEPOSITING,
                        () -> {
                            currentPath = scorePath.start();
                            scoreCount++;
                        })
                .state(State.DEPOSITING)
                .transition(() -> currentPath.isDone(), statesAfterDeposit[scoreCount-1],
                        () -> currentPath = pathsAfterDeposit[scoreCount-1].start())
                .state(State.RIGHT_INTAKE)
                .transition(() -> currentPath.isDone() || Robot.getInstance().intakeSwitch.pressed(),
                        State.LIFTING, () -> {
                            new BoxtubeRetractCommand().schedule();
                            new EndEffectorRetractCommand().schedule();
                            currentPath = liftingPath.start();
                        })
                .state(State.CENTER_INTAKE)
                .transition(() -> currentPath.isDone() || Robot.getInstance().intakeSwitch.pressed(),
                        State.LIFTING, () -> {
                            new BoxtubeRetractCommand().schedule();
                            new EndEffectorRetractCommand().schedule();
                            currentPath = liftingPath.start();
                        })
                .state(State.LEFT_INTAKE)
                .transition(() -> currentPath.isDone() || Robot.getInstance().intakeSwitch.pressed(),
                        State.LIFTING, () -> {
                            new BoxtubeRetractCommand().schedule();
                            new EndEffectorRetractCommand().schedule();
                            currentPath = liftingPath.start();
                        })
                .state(State.PARKING)
                .transition(() -> currentPath.isDone(), State.DONE)
                .state(State.DONE)
                .build();
    }

    @Override
    public void build() {
        liftingPath = new SampleLiftingPath().build();
        scorePath = new SampleScoreHighPath().build();

        rightIntakePath = new SampleIntakeRightPath().build();
        centerIntakePath = new SampleIntakeCenterPath().build();
        leftIntakePath = new SampleIntakeLeftPath().build();
        parkPath = new SampleParkPath().build();
    }

    @Override
    public void start() {
        sm.start();
        sm.setState(State.LIFTING);
        runtime = Globals.runtime;
    }

    @Override
    public void telemetry() {
        Telemetry tele = Globals.tele;
        tele.addLine("Four Sample Config");
        tele.addData("Config state", sm.getState());
        tele.addData("runtime", runtime.seconds());
    }

    @Override
    public Pose2d getStartPose() {
        return Globals.mapPose(40.5, 64, 270);
    }
}
