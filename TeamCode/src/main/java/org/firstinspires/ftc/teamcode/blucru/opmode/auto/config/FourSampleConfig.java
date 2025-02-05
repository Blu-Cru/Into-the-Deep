package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeCenterPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeLeftPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeRightPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleHighLiftPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleParkPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleHighDepositPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class FourSampleConfig extends AutoConfig {
    Path preloadLiftingPath, cycleLiftingPath, scorePath,
        rightFailsafePath, centerFailsafePath, leftFailsafePath;

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

//        statesAfterDeposit = new State[4];
//        statesAfterDeposit[0] = State.RIGHT_INTAKE;
//        statesAfterDeposit[1] = State.CENTER_INTAKE;
//        statesAfterDeposit[2] = State.LEFT_INTAKE;
//        statesAfterDeposit[3] = State.PARKING;

        pathsAfterDeposit = new Path[4];

        scoreCount = 0;

        sm = new StateMachineBuilder()
                .state(State.LIFTING)
                .onEnter(() -> logTransition(State.LIFTING))
                .transition(() -> Robot.getInstance().extension.getPIDError() < 5 && currentPath.isDone(),
                        State.DEPOSITING,
                        () -> {
                            currentPath = scorePath.start();
                        })
                .state(State.DEPOSITING)
                .onEnter(() -> logTransition(State.DEPOSITING))
                .transition(() -> currentPath.isDone() && scoreCount == 0, State.RIGHT_INTAKE,
                        () -> {
                            currentPath = pathsAfterDeposit[scoreCount].start();
                            scoreCount++;
                        })
                .transition(() ->currentPath.isDone() && scoreCount == 1, State.CENTER_INTAKE, () -> {
                    currentPath = pathsAfterDeposit[scoreCount].start();
                    scoreCount++;
                })
                .transition(() -> currentPath.isDone() && scoreCount == 2, State.LEFT_INTAKE, () -> {
                    currentPath = pathsAfterDeposit[scoreCount].start();
                    scoreCount++;
                })
                .transition(() -> currentPath.isDone() && scoreCount == 3, State.PARKING, () -> {
                    currentPath = pathsAfterDeposit[scoreCount].start();
                    scoreCount++;
                })
                .state(State.RIGHT_INTAKE)
                .onEnter(() -> logTransition(State.RIGHT_INTAKE))
                .transition(() -> currentPath.isDone() || Robot.getInstance().intakeSwitch.pressed(),
                        State.LIFTING, () -> {
                            new ClampGrabCommand().schedule();
                            new WheelStopCommand().schedule();
                            new ArmRetractCommand().schedule();
                            new BoxtubeRetractCommand().schedule();
                            currentPath = cycleLiftingPath.start();
                        })
                .state(State.CENTER_INTAKE)
                .onEnter(() -> logTransition(State.CENTER_INTAKE))
                .transition(() -> currentPath.isDone() || Robot.getInstance().intakeSwitch.pressed(),
                        State.LIFTING, () -> {
                            new ClampGrabCommand().schedule();
                            new WheelStopCommand().schedule();
                            new ArmRetractCommand().schedule();
                            new BoxtubeRetractCommand().schedule();
                            currentPath = cycleLiftingPath.start();
                        })
                .state(State.LEFT_INTAKE)
                .onEnter(() -> logTransition(State.LEFT_INTAKE))
                .transition(() -> currentPath.isDone() || Robot.getInstance().intakeSwitch.pressed(),
                        State.LIFTING, () -> {
                            new ClampGrabCommand().schedule();
                            new WheelStopCommand().schedule();
                            new ArmRetractCommand().schedule();
                            new BoxtubeRetractCommand().schedule();
                            currentPath = cycleLiftingPath.start();
                        })
                .state(State.PARKING)
                .onEnter(() -> logTransition(State.PARKING))
                .transition(() -> currentPath.isDone(), State.DONE)
                .state(State.DONE)
                .build();
    }

    @Override
    public void build() {
        preloadLiftingPath = new SampleHighLiftPath(0).build();
        cycleLiftingPath = new SampleHighLiftPath(600).build();

        scorePath = new SampleHighDepositPath().build();

        pathsAfterDeposit[0] = new SampleIntakeRightPath().build();
        pathsAfterDeposit[1] = new SampleIntakeCenterPath().build();
        pathsAfterDeposit[2] = new SampleIntakeLeftPath().build();
        pathsAfterDeposit[3] = new SampleParkPath().build();
    }

    @Override
    public void start() {
        scoreCount = 0;

        currentPath = preloadLiftingPath.start();

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
