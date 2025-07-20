package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.PivotRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.PreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleDriveToSubIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleIntakeAtPointPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleIntakeCenterPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleIntakeLeftPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleIntakeRightPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleHighLiftPreloadsPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleDepositHighFromSubPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleParkPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleHighDepositPreloadPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SampleParkFromSubIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

import java.util.ArrayList;
import java.util.List;

public class SampleCycleConfig extends AutoConfig {
    // TODO: make sure in the future there are no commands in these auto configs
    enum State{
        LIFTING_PRELOADS,
        DEPOSITING,
        RIGHT_INTAKE,
        CENTER_INTAKE,
        LEFT_INTAKE,
        RIGHT_FAILSAFE,
        CENTER_FAILSAFE,
        LEFT_FAILSAFE,
        DRIVING_TO_SUB_CYCLE,
        SCANNING_SUB,
        INTAKE_SUB,
        PARKING,
        DONE
    }

    int scoreCount;
    double scanTimeMillis;

    public SampleCycleConfig() {
        runtime = Globals.runtime;
        scoreCount = 0;

        sm = new StateMachineBuilder()
                .state(State.LIFTING_PRELOADS)
                .onEnter(() -> logTransition(State.LIFTING_PRELOADS))
                .transition(() -> Robot.getInstance().extension.getPIDError() < 7.0
                                && Robot.getInstance().extension.getDistance() > 13.0
                                && currentPath.isDone(),
                        State.DEPOSITING,
                        () -> {
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
                .transition(() -> currentPath.isDone() && scoreCount >= 3 && runtime.seconds() < 26.0, State.DRIVING_TO_SUB_CYCLE, () -> {
                    currentPath = new SampleDriveToSubIntakePath().start();
                    scoreCount++;
                })
                .transition(() -> currentPath.isDone() && runtime.seconds() > 26.0, State.PARKING, () -> {
                    currentPath = new SampleParkPath().start();
                })
                .state(State.RIGHT_INTAKE)
                .onEnter(() -> logTransition(State.RIGHT_INTAKE))
                .transition(() -> currentPath.isDone(), State.LIFTING_PRELOADS, () -> {
                    currentPath = new SampleHighLiftPreloadsPath().start();
                })
                .state(State.CENTER_INTAKE)
                .onEnter(() -> logTransition(State.CENTER_INTAKE))
                .transition(() -> currentPath.isDone(), State.LIFTING_PRELOADS, () -> {
                    currentPath = new SampleHighLiftPreloadsPath().start();
                })
                .state(State.LEFT_INTAKE)
                .onEnter(() -> logTransition(State.LEFT_INTAKE))
                .transition(() -> currentPath.isDone(), State.LIFTING_PRELOADS, () -> {
                    currentPath = new SampleHighLiftPreloadsPath().start();
                })
                .state(State.DRIVING_TO_SUB_CYCLE)
                .onEnter(() -> {
                    logTransition(State.DRIVING_TO_SUB_CYCLE);
                    Robot.getInstance().cvMaster.enableSampleDetector();
                })
                .transition(() -> currentPath.isDone(), State.SCANNING_SUB)
                .state(State.SCANNING_SUB)
                .onEnter(() -> {
                    logTransition(State.SCANNING_SUB);
                    scanTimeMillis = System.currentTimeMillis();
                })
                .transition(() -> System.currentTimeMillis() - scanTimeMillis > 300 && Robot.getInstance().cvMaster.sampleDetector.detectionList.hasSample(), State.INTAKE_SUB, () -> {
                    Pose2d blockPose = Robot.getInstance().cvMaster.sampleDetector.detectionList.getBestSamplePose();
//                    globalSamplePoses.remove(0);

                    Log.i("SampleCycleConfig", "got block pose at " + blockPose);
                    Log.i("SampleCycleConfig", "dt pose at " + Robot.getInstance().dt.pose);
                    currentPath = new SampleIntakeAtPointPath(Robot.getInstance().dt.pose, blockPose).start();
                })
                .transition(() -> runtime.seconds() > 29.6, State.DONE, () -> {
                    new FullRetractCommand().schedule();
                })
                .state(State.INTAKE_SUB)
                .onEnter(() -> logTransition(State.INTAKE_SUB))
                .transition(() -> currentPath.isDone() && Robot.validSample() && runtime.seconds() < 26.5, State.DEPOSITING, () -> {
                    Robot.getInstance().cvMaster.disableSampleDetector();
                    currentPath = new SampleDepositHighFromSubPath().start();
                })
                .transition(() -> currentPath.isDone() && Robot.validSample() && runtime.seconds() > 26.5, State.PARKING, () -> {
                    new FullRetractCommand().schedule();
                    currentPath = new SampleParkFromSubIntakePath().start();
                })
                .transition(() -> currentPath.isDone() && !Robot.validSample() && runtime.seconds() < 28.5, State.SCANNING_SUB, () -> {
                    new SequentialCommandGroup(
                            new PreIntakeCommand(),
                            new WaitCommand(400),
                            new BoxtubeRetractCommand()
                    ).schedule();
                })
                .transition(()  -> currentPath.isDone() && !Robot.validSample() && runtime.seconds() > 28.5, State.DONE, () -> {
                    new FullRetractCommand().schedule();
                })
                .state(State.PARKING)
                .onEnter(() -> logTransition(State.PARKING))
                .transition(() -> currentPath.isDone(), State.DONE)
                .state(State.DONE)
                .build();
    }

    @Override
    public void build() {
        Robot.getInstance().cvMaster.stop();

        Robot.getInstance().cvMaster.startSampleStreaming();
        Robot.getInstance().cvMaster.disableSampleDetector();
    }

    @Override
    public void start() {
        scoreCount = 0;

        currentPath = new SampleHighLiftPreloadsPath().start();

        sm.start();
        sm.setState(State.LIFTING_PRELOADS);
        runtime = Globals.runtime;
        runtime.reset();
    }

    @Override
    public void telemetry() {
        Telemetry tele = Globals.tele;
        tele.addLine("Sample Cycle Config");
        tele.addData("Config state", sm.getState());
        tele.addData("runtime", runtime.seconds());
    }

    @Override
    public Pose2d getStartPose() {
        return Globals.mapPose(-40, -64, 90);
    }
}
