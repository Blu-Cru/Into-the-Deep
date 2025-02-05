package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeCenterPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeLeftPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeRightPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleHighLiftPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleParkPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SamplePartnerIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleHighDepositPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class FiveSampleConfig extends AutoConfig {
    Path liftingPath, scorePath,
            rightIntakePath, centerIntakePath, leftIntakePath, partnerIntakePath,
            rightFailsafePath, centerFailsafePath, leftFailsafePath, partnerFailsafePath,
            parkPath;

    enum State{
        LIFTING,
        DEPOSITING,
        RIGHT_INTAKE,
        CENTER_INTAKE,
        LEFT_INTAKE,
        PARTNER_INTAKE,
        RIGHT_FAILSAFE,
        CENTER_FAILSAFE,
        LEFT_FAILSAFE,
        PARKING,
        DONE
    }

    public State[] statesAfterDeposit;
    public Path[] pathsAfterDeposit;

    int scoreCount;

    public FiveSampleConfig() {
        runtime = Globals.runtime;

        statesAfterDeposit = new State[4];
        statesAfterDeposit[0] = State.RIGHT_INTAKE;
        statesAfterDeposit[1] = State.CENTER_INTAKE;
        statesAfterDeposit[2] = State.LEFT_INTAKE;
        statesAfterDeposit[3] = State.PARTNER_INTAKE;
        statesAfterDeposit[4] = State.PARKING;

        pathsAfterDeposit = new Path[4];
        pathsAfterDeposit[0] = rightIntakePath;
        pathsAfterDeposit[1] = centerIntakePath;
        pathsAfterDeposit[2] = leftIntakePath;
        pathsAfterDeposit[3] = partnerIntakePath;
        pathsAfterDeposit[4] = parkPath;

        scoreCount = 0;

        sm = new StateMachineBuilder()
                .state(State.LIFTING)
                .transition(() -> Robot.getInstance().extension.getPIDError() < 2 && currentPath.isDone(),
                        State.DEPOSITING,
                        () -> {
                            currentPath = scorePath.start();
                            scoreCount++;
                        })
                .state(State.DEPOSITING)
                .transition(() -> currentPath.isDone(), statesAfterDeposit[scoreCount-1],
                        () -> currentPath = pathsAfterDeposit[scoreCount-1].start())
                .state(State.RIGHT_INTAKE)
                .transition(() -> currentPath.isDone() || Robot.getInstance().intakeSwitch.pressed(),
                        State.LIFTING, () -> {
                            new BoxtubeRetractCommand().schedule();
                            new EndEffectorRetractCommand().schedule();
                            currentPath = liftingPath.start();
                        })
                .state(State.CENTER_INTAKE)
                .transition(() -> currentPath.isDone() || Robot.getInstance().intakeSwitch.pressed(),
                        State.LIFTING, () -> {
                            new BoxtubeRetractCommand().schedule();
                            new EndEffectorRetractCommand().schedule();
                            currentPath = liftingPath.start();
                        })
                .state(State.LEFT_INTAKE)
                .transition(() -> currentPath.isDone() || Robot.getInstance().intakeSwitch.pressed(),
                        State.LIFTING, () -> {
                            new BoxtubeRetractCommand().schedule();
                            new EndEffectorRetractCommand().schedule();
                            currentPath = liftingPath.start();
                        })
                .state(State.PARTNER_INTAKE)
                .transition(() -> currentPath.isDone() || Robot.getInstance().intakeSwitch.pressed(),
                        State.LIFTING, () -> {
                            new BoxtubeRetractCommand().schedule();
                            new EndEffectorRetractCommand().schedule();
                            currentPath = liftingPath.start();
                        })
                .state(State.PARKING)
                .transition(() -> currentPath.isDone(), State.DONE)
                .state(State.DONE)
                .build();
    }

    @Override
    public void build() {
        liftingPath = new SampleHighLiftPath().build();
        scorePath = new SampleHighDepositPath().build();

        rightIntakePath = new SampleIntakeRightPath().build();
        centerIntakePath = new SampleIntakeCenterPath().build();
        leftIntakePath = new SampleIntakeLeftPath().build();
        partnerIntakePath = new SamplePartnerIntakePath().build();
        parkPath = new SampleParkPath().build();
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
        tele.addLine("Five Sample Config");
        tele.addData("Config state", sm.getState());
        tele.addData("runtime", runtime.seconds());
    }

    @Override
    public Pose2d getStartPose() {
        return Globals.mapPose(-40.5, -64, 90);
    }
}
