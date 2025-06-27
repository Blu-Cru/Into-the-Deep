package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleIntakeCenterPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleIntakeLeftPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleIntakeRightPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleHighLiftPreloadsPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleParkPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleHighDepositPreloadPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class FourSampleConfig extends AutoConfig {
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

    int scoreCount;

    public FourSampleConfig() {
        runtime = Globals.runtime;

        scoreCount = 0;

        sm = new StateMachineBuilder()
                .state(State.LIFTING)
                .onEnter(() -> logTransition(State.LIFTING))
                .transition(() -> Robot.getInstance().extension.getPIDError() < 5
                                && Robot.getInstance().extension.getDistance() > 15.0
                                && currentPath.isDone(),
                        State.DEPOSITING, () -> {
                    currentPath = new SampleHighDepositPreloadPath().start();
                })
                .state(State.DEPOSITING)
                .onEnter(() -> logTransition(State.DEPOSITING))
                .transition(() -> currentPath.isDone() && scoreCount == 0, State.RIGHT_INTAKE, () -> {
                    currentPath = new SampleIntakeRightPath().start();
                    scoreCount++;
                })
                .transition(() -> currentPath.isDone() && scoreCount == 1, State.CENTER_INTAKE, () -> {
                    currentPath = new SampleIntakeCenterPath().start();
                    scoreCount++;
                })
                .transition(() -> currentPath.isDone() && scoreCount == 2, State.LEFT_INTAKE, () -> {
                    currentPath = new SampleIntakeLeftPath().start();
                    scoreCount++;
                })
                .transition(() -> currentPath.isDone() && scoreCount == 3, State.PARKING, () -> {
                    currentPath = new SampleParkPath().start();
                    scoreCount++;
                })
                .state(State.RIGHT_INTAKE)
                .onEnter(() -> logTransition(State.RIGHT_INTAKE))
                .transition(() -> currentPath.isDone(), State.LIFTING, () -> {
                    currentPath = new SampleHighLiftPreloadsPath().start();
                })
                .state(State.CENTER_INTAKE)
                .onEnter(() -> logTransition(State.CENTER_INTAKE))
                .transition(() -> currentPath.isDone(), State.LIFTING, () -> {
                    currentPath = new SampleHighLiftPreloadsPath().start();
                })
                .state(State.LEFT_INTAKE)
                .onEnter(() -> logTransition(State.LEFT_INTAKE))
                .transition(() -> currentPath.isDone(), State.LIFTING, () -> {
                    currentPath = new SampleHighLiftPreloadsPath().start();
                })
                .state(State.PARKING)
                .onEnter(() -> logTransition(State.PARKING))
                .transition(() -> currentPath.isDone(), State.DONE)
                .state(State.DONE)
                .build();
    }

    @Override
    public void build() {
    }

    @Override
    public void start() {
        scoreCount = 0;

        currentPath = new SampleHighLiftPreloadsPath().start();

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
        return Globals.mapPose(-40.5, -64, 90);
    }
}
