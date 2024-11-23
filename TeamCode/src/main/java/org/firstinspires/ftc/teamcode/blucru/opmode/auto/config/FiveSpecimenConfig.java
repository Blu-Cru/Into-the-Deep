package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.PivotCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmPreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.CollectCenterBlockPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.CollectLeftBlockPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.CollectRightBlockPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.SpecimenCycleDepositPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.SpecimenCycleIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.SpecimenParkIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.SpecimenPreloadDepositPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.SpitPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class FiveSpecimenConfig extends AutoConfig {
    int scoreCount;
    int spitCount = 0;
    Path[] collectPaths;

    enum State {
        LIFTING_PRELOAD,
        SCORING_PRELOAD,
        COLLECTING_BLOCKS,
        SPITTING,
        INTAKING_CYCLE,
        DEPOSIT_CYCLE,
        SCORING_CYCLE,
        PARK_INTAKING,
        PARKED
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
                .transition(() -> currentPath.isDone() || Robot.getInstance().intakeSwitch.pressed() && Robot.getInstance().pivot.getAngle() < 0.4,
                        State.SPITTING, () -> {
                            new ArmPreIntakeCommand().schedule();
                            new ClampGrabCommand().schedule();
                            new WheelStopCommand().schedule();
                            new ExtensionCommand(4).schedule();
                            currentPath = new SpitPath().build().start();
                        })
                .state(State.SPITTING)
                .transition(() -> currentPath.isDone() && spitCount < 2, State.COLLECTING_BLOCKS, () -> {
                    spitCount++;
                    currentPath = collectPaths[spitCount];
                })
                .transition(() -> currentPath.isDone() && spitCount >= 2, State.INTAKING_CYCLE, () -> {
                    currentPath = new SpecimenCycleIntakePath().build().start();
                })

                .state(State.INTAKING_CYCLE)
                .transition(() -> currentPath.isDone() || (Robot.getInstance().intakeSwitch.pressed() && Robot.getInstance().pivot.getAngle() < 0.35),
                        State.DEPOSIT_CYCLE, () -> {
                    new SequentialCommandGroup(
                            new PivotCommand(1.2),
                            new WaitCommand(300),
                            new SpecimenBackCommand()
                    ).schedule();

                    currentPath = new SpecimenCycleDepositPath(scoreCount).build().start();
                })

                .state(State.DEPOSIT_CYCLE)
                .transition(() -> currentPath.isDone() && scoreCount < 4 && runtime.seconds() < 25,
                        State.INTAKING_CYCLE,
                        () -> {
                            scoreCount++;
                            currentPath = new SpecimenCycleIntakePath().build().start();
                        })
                .transition(() -> currentPath.isDone() && !(scoreCount < 4 && runtime.seconds() < 25), State.PARK_INTAKING, () -> {
                    Log.i("Five Specimen Config", "parking, time = " + runtime.seconds());
                    currentPath = new SpecimenParkIntakePath().build().start();
                })
                .state(State.PARK_INTAKING)
                .transition(() -> currentPath.isDone() || (Robot.getInstance().intakeSwitch.pressed() && Robot.getInstance().pivot.getAngle() < 0.35),
                        State.PARKED, () -> {
                            new ClampGrabCommand().schedule();
                            new WheelStopCommand().schedule();
                        })
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

    }

    @Override
    public Pose2d getStartPose() {
        return Globals.mapPose(7.5, -64, 90);
    }
}
