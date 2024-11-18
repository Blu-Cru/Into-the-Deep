package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeRightPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleLiftingPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleScoreHighPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class FourSampleConfig extends AutoConfig {
    Path liftingPath, scorePath,
        rightIntakePath, centerIntakePath, leftIntakePath,
        rightFailsafePath, centerFailsafePath, leftFailsafePath;

    Path currentPath;
    StateMachine sm;

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

    ElapsedTime runtime;
    int scoreCount;

    public FourSampleConfig() {
        runtime = Globals.runtime;

        scoreCount = 0;

        sm = new StateMachineBuilder()
                .state(State.LIFTING)
                .transition(() -> Robot.getInstance().extension.getPIDError() < 2 && currentPath.isDone(),
                        State.DEPOSITING,
                        () -> currentPath = scorePath.start())
                .state(State.DEPOSITING)
                .transition(() -> currentPath.isDone())
                .build();
    }

    @Override
    public void build() {
        liftingPath = new SampleLiftingPath().build();
        scorePath = new SampleScoreHighPath().build();

        rightIntakePath = new SampleIntakeRightPath().build();
    }

    @Override
    public void start() {
        sm.start();
        sm.setState(State.LIFTING);
        runtime = Globals.runtime;
    }

    @Override
    public void run() {
        sm.update();
        currentPath.run();
    }

    @Override
    public void telemetry() {
        Telemetry tele = Globals.tele;
        tele.addLine("Four Sample Config");
        tele.addData("State", sm.getState());
        tele.addData("runtime", runtime.seconds());
    }
}
