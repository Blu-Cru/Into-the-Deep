package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.ExtensionCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmPreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenBackDunkRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontDunkCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.specimen.SpecimenFrontDunkRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.CollectCenterBlockPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.CollectLeftBlockPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.specimen.CollectRightBlockPath;
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
        LIFTING_CYCLE
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
                    currentPath = new CollectLeftBlockPath().build().start();
                })

                .state(State.COLLECTING_BLOCKS)
                .transition(() -> currentPath.isDone() || Robot.getInstance().intakeSwitch.pressed(),
                        State.SPITTING, () -> {
                            new ArmPreIntakeCommand().schedule();
                            new ClampGrabCommand().schedule();
                            new WheelStopCommand().schedule();
                            new ExtensionCommand(4).schedule();
                            currentPath = new SpitPath().build().start();
                        })
                .state(State.SPITTING)
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
