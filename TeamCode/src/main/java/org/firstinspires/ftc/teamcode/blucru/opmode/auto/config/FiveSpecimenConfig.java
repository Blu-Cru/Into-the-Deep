package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontDunkRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.SpecimenPreloadDepositPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class FiveSpecimenConfig extends AutoConfig {
    int scoreCount;

    enum State {
        LIFTING_PRELOAD,
        SCORING_PRELOAD,
        COLLECTING_BLOCKS,
        INTAKING_CYCLE,
        LIFTING_CYCLE
    }

    public FiveSpecimenConfig() {
        runtime = Globals.runtime;
        scoreCount = 0;

        sm = new StateMachineBuilder()
                .state(State.LIFTING_PRELOAD)
                .transition(() -> currentPath.isDone() && Robot.getInstance().extension.getPIDError() < 2, State.SCORING_PRELOAD)
                .state(State.SCORING_PRELOAD)
                .onEnter(() -> {
                    if(scoreCount == 0) {
                        new SequentialCommandGroup(
                                new SpecimenFrontDunkCommand(),
                                new WaitCommand(500),
                                new ClampReleaseCommand(),
                                new WheelReverseCommand(),
                                new WaitCommand(400),
                                new SpecimenFrontDunkRetractCommand()
                        ).schedule();
                    } else {
                        new SequentialCommandGroup(
                                new SpecimenBackDunkCommand(),
                                new WaitCommand(500),
                                new ClampReleaseCommand(),
                                new WheelReverseCommand(),
                                new WaitCommand(400),
                                new SpecimenBackDunkRetractCommand()
                        ).schedule();
                    }
                })
                .transitionTimed(700, State.COLLECTING_BLOCKS, () -> {
//                    currentPath = new CollectingBlocksPath().build.start();
                })

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
        runtime = Globals.runtime;
    }

    @Override
    public void telemetry() {

    }

    @Override
    public Pose2d getStartPose() {
        return Globals.mapPose(10, -64, 90);
    }
}
