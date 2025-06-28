package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleIntakeAtPointPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.CollectCenterBlockPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.CollectLeftBlockPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.CollectRightBlockPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.DepositSubSamplePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenCycleIntakeFailsafePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenDepositPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenParkIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenPreloadDepositPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpitIntakeSpecPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpitPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class SixSpecimenConfig extends AutoConfig {
    int scoreCount;
    int spitCount = 0, thisCycleIntakeFailCount = 0;
    Path[] collectPaths;

    enum State {
        LIFTING_PRELOAD,
        SCANNING_SUB,
        COLLECTING_SUB_SAMPLE,
        DROPPING_SUB_SAMPLE_AT_HUMAN_PLAYER,
        COLLECTING_BLOCKS,
        SPITTING,
        INTAKING_CYCLE,
        CONFIRMING_INTAKE,
        DEPOSIT_CYCLE,
        PARK_INTAKING,
        PARKED,

        INTAKE_FAILSAFE_CYCLE
    }

    double scanTimeMillis;
    double attemptedIntakeTriesInSub = 0;
    public SixSpecimenConfig() {
        runtime = Globals.runtime;
        scoreCount = 0;

        collectPaths = new Path[3];
        collectPaths[0] = new CollectLeftBlockPath().build();
        collectPaths[1] = new CollectCenterBlockPath().build();
        collectPaths[2] = new CollectRightBlockPath().build();

        sm = new StateMachineBuilder()
                .state(State.LIFTING_PRELOAD)
                .transition(() -> currentPath.isDone(), State.SCANNING_SUB, () -> {
                    scoreCount++;
                    logTransition(State.SCANNING_SUB);
                    scanTimeMillis = System.currentTimeMillis();
                })

                .state(State.SCANNING_SUB)
                .transition(() -> System.currentTimeMillis() - scanTimeMillis > 300
                                && Robot.getInstance().cvMaster.sampleDetector.detectionList.hasSpec(),
                        State.COLLECTING_SUB_SAMPLE, () -> {
                            Pose2d blockPose = Robot.getInstance().cvMaster.sampleDetector.detectionList.getBestSpecPose();

                            Log.i("SixSpecimenConfig", "got block pose at " + blockPose);
                            Log.i("SixSpecimenConfig", "dt pose at " + Robot.getInstance().dt.pose);
                            currentPath = new SampleIntakeAtPointPath(Robot.getInstance().dt.pose, blockPose).start();
                            attemptedIntakeTriesInSub++;
                        })
                .transitionTimed(1.0, State.COLLECTING_BLOCKS, () -> {
                    Robot.getInstance().cvMaster.disableSampleDetector();
                    currentPath = new CollectLeftBlockPath().start();
                })

                .state(State.COLLECTING_SUB_SAMPLE)
                .transition(() -> currentPath.isDone() && Robot.validSample(), State.DROPPING_SUB_SAMPLE_AT_HUMAN_PLAYER, () -> {
                    logTransition(State.DROPPING_SUB_SAMPLE_AT_HUMAN_PLAYER);
                    Robot.getInstance().cvMaster.disableSampleDetector();
                    currentPath = new DepositSubSamplePath().build().start();
                })
                .transition(() -> currentPath.isDone() && !Robot.validSample() && attemptedIntakeTriesInSub < 2, State.SCANNING_SUB, () -> {
                    new BoxtubeRetractCommand().schedule();
                })
                .transition(() -> currentPath.isDone() && !Robot.validSample() && attemptedIntakeTriesInSub >= 2, State.COLLECTING_BLOCKS, () -> {
                    logTransition(State.COLLECTING_BLOCKS);
                    currentPath = new CollectLeftBlockPath().build().start();
                })

                .state(State.DROPPING_SUB_SAMPLE_AT_HUMAN_PLAYER)
                .transition(() -> currentPath.isDone(), State.COLLECTING_BLOCKS, () -> {
                    currentPath = new CollectLeftBlockPath().start();
                })

                .state(State.COLLECTING_BLOCKS)
                .transition(() -> currentPath.isDone() && spitCount < 2, State.SPITTING, () -> {
                    spitCount++;
                    currentPath = new SpitPath().build().start();
                })
                .transition(() -> currentPath.isDone() && spitCount >= 2, State.INTAKING_CYCLE, () -> {
                    currentPath = new SpitIntakeSpecPath(35).build().start();
                })
                .state(State.SPITTING)
                .transition(() -> currentPath.isDone(), State.COLLECTING_BLOCKS, () -> {
                    currentPath = collectPaths[spitCount].start();
                })
                .state(State.INTAKING_CYCLE)
                .transition(() -> (currentPath.isDone() && Robot.getInstance().extension.getDistance() < 8.0 &&
                                (Robot.validSample() || thisCycleIntakeFailCount >= 1)),
                        State.DEPOSIT_CYCLE, () -> {
                            currentPath = new SpecimenDepositPath(-1).start();
                        })
                .transition(() -> (currentPath.isDone() && thisCycleIntakeFailCount < 1 && !Robot.validSample()), State.INTAKE_FAILSAFE_CYCLE, () -> {
                    currentPath = new SpecimenCycleIntakeFailsafePath(scoreCount).build().start();
                    thisCycleIntakeFailCount++;
                })

                .state(State.INTAKE_FAILSAFE_CYCLE)
                .transition(() -> Robot.validSample(), State.DEPOSIT_CYCLE, () -> {
                    currentPath = new SpecimenDepositPath(-1).start();
                })
                .transition(() -> currentPath.isDone(), State.INTAKING_CYCLE, () -> {
                    currentPath = new SpecimenIntakePath().build().start();
                })

                .state(State.DEPOSIT_CYCLE)
                .transition(() -> currentPath.isDone() && scoreCount < 5 && runtime.seconds() < 25, State.INTAKING_CYCLE, () -> {
                    thisCycleIntakeFailCount = 0;
                    scoreCount++;
                    currentPath = new SpecimenIntakePath().build().start();
                })
                .transition(() -> currentPath.isDone() && !(scoreCount < 5 && runtime.seconds() < 25), State.PARK_INTAKING, () -> {
                    Log.i("Five Specimen Config", "parking, time = " + runtime.seconds());
                    currentPath = new SpecimenParkIntakePath().build().start();
                })
                .onExit(() -> thisCycleIntakeFailCount = 0)

                .state(State.PARK_INTAKING)
                .transition(() -> currentPath.isDone(), State.PARKED)
                .state(State.PARKED)
                .build();
    }

    @Override
    public void build() {
        Robot.getInstance().cvMaster.stop();

        Robot.getInstance().cvMaster.startSampleStreaming();
    }

    @Override
    public void start() {
        scoreCount = 0;
        currentPath = new SpecimenPreloadDepositPath(300, 5).build().start();
        Robot.getInstance().cvMaster.enableSampleDetector();
        sm.setState(FiveSpecimenConfig.State.LIFTING_PRELOAD);
        sm.start();
        Globals.runtime = new ElapsedTime();
        runtime = Globals.runtime;
    }

    @Override
    public void telemetry() {
        Telemetry tele = Globals.tele;

        tele.addData("State", sm.getState());
        tele.addData("Score Count", scoreCount);
        tele.addData("Spit Count", spitCount);
        tele.addData("Cycle Intake Fail Count", thisCycleIntakeFailCount);
    }

    @Override
    public Pose2d getStartPose() {
        return Globals.mapPose(7.5, -64, 90);
    }
}
