package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.CollectCenterBlockPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.CollectLeftBlockPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.CollectRightBlockPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenDepositPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenCycleIntakeFailsafePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenParkIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenPreloadDepositPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpitPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpitIntakeSpecPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class FiveSpecimenConfig extends AutoConfig {
    int scoreCount;
    int spitCount = 0, thisCycleIntakeFailCount = 0;
    Path[] collectPaths;

    enum State {
        LIFTING_PRELOAD,
        COLLECTING_BLOCKS,
        SPITTING,
        INTAKING_CYCLE,
        CONFIRMING_INTAKE,
        DEPOSIT_CYCLE,
        PARK_INTAKING,
        PARKED,

        INTAKE_FAILSAFE_CYCLE
    }

    public FiveSpecimenConfig() {
        runtime = Globals.runtime;
        scoreCount = 0;

        collectPaths = new Path[3];
        collectPaths[0] = new CollectLeftBlockPath().build();
        collectPaths[1] = new CollectCenterBlockPath().build();
        collectPaths[2] = new CollectRightBlockPath().build();

        sm = new StateMachineBuilder()
                .state(State.LIFTING_PRELOAD)
                .transition(() -> currentPath.isDone(), State.COLLECTING_BLOCKS, () -> {
                    scoreCount++;
                    currentPath = new CollectLeftBlockPath().build().start();
                })
                .state(State.COLLECTING_BLOCKS)
                .transition(() -> currentPath.isDone() && spitCount < 2, State.SPITTING, () -> {
                    spitCount++;
                    currentPath = new SpitPath().build().start();
                })
                .transition(() -> currentPath.isDone() && spitCount >= 2, State.INTAKING_CYCLE, () -> {
                    currentPath = new SpitIntakeSpecPath().build().start();
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
                .transition(() -> currentPath.isDone() && scoreCount < 4 && runtime.seconds() < 25, State.INTAKING_CYCLE, () -> {
                    thisCycleIntakeFailCount = 0;
                    scoreCount++;
                    currentPath = new SpecimenIntakePath().build().start();
                })
                .transition(() -> currentPath.isDone() && !(scoreCount < 4 && runtime.seconds() < 25), State.PARK_INTAKING, () -> {
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

    }

    @Override
    public void start() {
        scoreCount = 0;
        currentPath = new SpecimenPreloadDepositPath().build().start();
        sm.setState(State.LIFTING_PRELOAD);
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
