package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleHighLiftPreloadsPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleHighDepositPreloadPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleLowDepositPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleLowLiftPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.CollectCenterBlockPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.CollectLeftBlockPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.CollectRightBlockPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.CrossDepositYellowSamplePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenDepositPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenCycleIntakeFailsafePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenParkIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenParkYellowDeposit;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpecimenPreloadDepositPath;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.specimen.SpitPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class FourSpecimenOneSampleConfig extends AutoConfig {
    int scoreCount;
    int spitCount = 0, thisCycleIntakeFailCount = 0;
    Path[] collectPaths;
    double latestHighLiftTime = 27.8;
    double latestLowLiftTime = 28.3;

    enum State {
        LIFTING_PRELOAD,
        COLLECTING_BLOCKS,
        SPITTING,
        INTAKING_CYCLE,
        DEPOSIT_CYCLE,
        INTAKING_YELLOW,
        CROSSING_WITH_YELLOW,
        LIFTING_YELLOW_HIGH,
        LIFTING_YELLOW_LOW,
        DEPOSIT_YELLOW,
        PARKING,
        PARKED,

        INTAKE_FAILSAFE_CYCLE,
        INTAKE_YELLOW_FAILSAFE
    }

    public FourSpecimenOneSampleConfig() {
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
                .transition(() -> currentPath.isDone() || (
                                Robot.validSample()
                                        && Robot.getInstance().pivot.getAngle() < 0.4
                                        && Math.abs(Robot.getInstance().getBoxtubePoint3d().y) < 44
                        ),
                        State.SPITTING, () -> {
//                            new ArmPreIntakeCommand().schedule();
//                            new ClampGrabCommand().schedule();
//                            new WheelStopCommand().schedule();
                            new ExtensionCommand(4).schedule();
                            currentPath = new SpitPath().build().start();
                        })
                .state(State.SPITTING)
                .transition(() -> currentPath.isDone() && spitCount < 2, State.COLLECTING_BLOCKS, () -> {
                    spitCount++;
                    currentPath = collectPaths[spitCount].start();
                })
                .transition(() -> currentPath.isDone() && spitCount >= 2, State.INTAKING_CYCLE, () -> {
                    currentPath = new SpecimenIntakePath().build().start();
                })

                .state(State.INTAKING_CYCLE)
                .transition(() -> (currentPath.isDone() && thisCycleIntakeFailCount >= 1)
                                || (Robot.validSample()
                                && Robot.getInstance().pivot.getAngle() < 0.55)
                                && Robot.getInstance().getBoxtubePose().getY() > 5,
                        State.DEPOSIT_CYCLE, () -> {
                            currentPath = new SpecimenDepositPath().build().start();
                        })
                .transition(() -> (currentPath.isDone() && thisCycleIntakeFailCount < 1), State.INTAKE_FAILSAFE_CYCLE, () -> {
                    currentPath = new SpecimenCycleIntakeFailsafePath(scoreCount).build().start();
                    thisCycleIntakeFailCount++;
                })

                .state(State.INTAKE_FAILSAFE_CYCLE)
                .transition(() -> Robot.validSample(), State.DEPOSIT_CYCLE, () -> {
                    currentPath = new SpecimenDepositPath().build().start();
                })
                .transition(() -> currentPath.isDone(), State.INTAKING_CYCLE, () -> {
                    currentPath = new SpecimenIntakePath().build().start();
                })

                .state(State.DEPOSIT_CYCLE)
                .transition(() -> currentPath.isDone() && scoreCount < 3 && runtime.seconds() < 25,
                        State.INTAKING_CYCLE,
                        () -> {
                            thisCycleIntakeFailCount = 0;
                            scoreCount++;
                            currentPath = new SpecimenIntakePath().build().start();
                        })
                .transition(() -> currentPath.isDone() && !(scoreCount < 3 && runtime.seconds() < 25), State.INTAKING_YELLOW, () -> {
                    Log.i("Five Specimen Config", "parking, time = " + runtime.seconds());
                    currentPath = new SpecimenParkIntakePath().build().start();
                })
                .onExit(() -> thisCycleIntakeFailCount = 0)

                .state(State.INTAKING_YELLOW)
                .transition(() -> Robot.validSample() && Robot.getInstance().pivot.getAngle() < 0.35,
                        State.CROSSING_WITH_YELLOW, () -> {
                            new FullRetractCommand().schedule();
                            currentPath = new CrossDepositYellowSamplePath().start();
                        })
                .transition(() -> currentPath.isDone(), State.INTAKE_YELLOW_FAILSAFE)
                .state(State.INTAKE_YELLOW_FAILSAFE)
                .onEnter(() -> {
                    new ExtensionCommand(3).schedule();
                })
                .transition(() -> Robot.validSample() && Robot.getInstance().pivot.getAngle() < 0.35, State.CROSSING_WITH_YELLOW, () -> {
                    new FullRetractCommand().schedule();
                    currentPath = new CrossDepositYellowSamplePath().start();
                })
                .transitionTimed(0.5, State.INTAKING_YELLOW)

                .state(State.CROSSING_WITH_YELLOW)
                .transition(() -> currentPath.isDone() && Robot.getInstance().cvMaster.seesSampleTag() && runtime.seconds() < latestHighLiftTime, State.LIFTING_YELLOW_HIGH, () -> {
                    Robot.getInstance().dt.updateAprilTags();
                    currentPath = new SampleHighLiftPreloadsPath().start();
                })
                .transition(() -> currentPath.isDone() && Robot.getInstance().cvMaster.seesSampleTag() && runtime.seconds() > latestHighLiftTime && runtime.seconds() < latestLowLiftTime, State.LIFTING_YELLOW_LOW, () -> {
                    currentPath = new SampleLowLiftPath().start();
                })
                .transition(() -> runtime.seconds() > latestLowLiftTime, State.PARKED, () -> new FullRetractCommand().schedule())

                .state(State.LIFTING_YELLOW_HIGH)
                .transition(() -> currentPath.isDone() && Robot.getInstance().extension.getDistance() > 22, State.DEPOSIT_YELLOW, () -> {
                    currentPath = new SampleHighDepositPreloadPath().start();
                })

                .state(State.LIFTING_YELLOW_LOW)
                .transition(() -> currentPath.isDone() && Robot.getInstance().extension.getDistance() > 5, State.DEPOSIT_YELLOW, () -> {
                    currentPath = new SampleLowDepositPath().start();
                })

                .state(State.DEPOSIT_YELLOW)
                .transition(() -> currentPath.isDone(), State.PARKING, () -> {
                    currentPath = new SpecimenParkYellowDeposit().start();
                })

                .state(State.PARKING)
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
