package org.firstinspires.ftc.teamcode.blucru.opmode.auto.config;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.RetractFromVerticalIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.boxtube.BoxtubeRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleDriveToSubIntakePath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeAtPointPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeCenterPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeLeftPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeRightPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleHighLiftPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleLiftHighFromSubPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleParkPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleHighDepositPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleSubIntakeFailPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.auto.AutoConfig;

public class SampleCycleConfig extends AutoConfig {
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
        DRIVING_TO_SUB_CYCLE,
        SCANNING_SUB,
        INTAKE_SUB,
        INTAKE_SUB_FAIL,
        PARKING,
        DONE
    }

    public Path[] pathsAfterDeposit;

    int scoreCount;
    double scanTimeMillis;

    public SampleCycleConfig() {
        runtime = Globals.runtime;
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
                .transition(() -> currentPath.isDone() && scoreCount == 1, State.CENTER_INTAKE, () -> {
                    currentPath = pathsAfterDeposit[scoreCount].start();
                    scoreCount++;
                })
                .transition(() -> currentPath.isDone() && scoreCount == 2, State.LEFT_INTAKE, () -> {
                    currentPath = pathsAfterDeposit[scoreCount].start();
                    scoreCount++;
                })
                .transition(() -> currentPath.isDone() && scoreCount >= 3 && runtime.seconds() < 26.0, State.DRIVING_TO_SUB_CYCLE, () -> {
                    currentPath = new SampleDriveToSubIntakePath().start();
                    scoreCount++;
                })
                .state(State.RIGHT_INTAKE)
                .onEnter(() -> logTransition(State.RIGHT_INTAKE))
                .transition(() -> currentPath.isDone() || Robot.justValidSample(),
                        State.LIFTING, () -> {
                            new ClampGrabCommand().schedule();
                            new WheelStopCommand().schedule();
                            new ArmRetractCommand().schedule();
                            new BoxtubeRetractCommand().schedule();
                            currentPath = cycleLiftingPath.start();
                        })
                .state(State.CENTER_INTAKE)
                .onEnter(() -> logTransition(State.CENTER_INTAKE))
                .transition(() -> currentPath.isDone() || Robot.justValidSample(),
                        State.LIFTING, () -> {
                            new ClampGrabCommand().schedule();
                            new WheelStopCommand().schedule();
                            new ArmRetractCommand().schedule();
                            new BoxtubeRetractCommand().schedule();
                            currentPath = cycleLiftingPath.start();
                        })
                .state(State.LEFT_INTAKE)
                .onEnter(() -> logTransition(State.LEFT_INTAKE))
                .transition(() -> currentPath.isDone() || Robot.justValidSample(),
                        State.LIFTING, () -> {
                            new ClampGrabCommand().schedule();
                            new WheelStopCommand().schedule();
                            new ArmRetractCommand().schedule();
                            new BoxtubeRetractCommand().schedule();
                            currentPath = cycleLiftingPath.start();
                        })
                .state(State.DRIVING_TO_SUB_CYCLE)
                .onEnter(() -> Robot.getInstance().cvMaster.enableSampleDetector())
                .transition(() -> currentPath.isDone(), State.SCANNING_SUB)
                .state(State.SCANNING_SUB)
                .onEnter(() -> {
                    scanTimeMillis = System.currentTimeMillis();
                })
                .transition(() -> System.currentTimeMillis() - scanTimeMillis > 500 && System.currentTimeMillis() - scanTimeMillis < 700 && Robot.getInstance().cvMaster.sampleDetector.hasValidDetection(), State.INTAKE_SUB, () -> {
                    Pose2d blockPose = Robot.getInstance().cvMaster.sampleDetector.getGlobalPose(
                            Robot.getInstance().dt.pose);
                    currentPath = new SampleIntakeAtPointPath(Robot.getInstance().dt.pose.vec(), blockPose).start();
                })
                .transitionTimed(1.0, State.INTAKE_SUB_FAIL, () -> {
                    currentPath = new SampleSubIntakeFailPath().start();
                })
                .state(State.INTAKE_SUB)
                .transition(() -> Robot.justValidSample() && runtime.seconds() < 27.5, State.LIFTING, () -> {
                    Robot.getInstance().cvMaster.disableSampleDetector();
                    new RetractFromVerticalIntakeCommand().schedule();
                    currentPath = new SampleLiftHighFromSubPath().start();
                })
                .transition(() -> currentPath.isDone(), State.INTAKE_SUB_FAIL, () -> {
                    currentPath = new SampleSubIntakeFailPath().start();
                })
                .state(State.INTAKE_SUB_FAIL)
                .transition(() -> currentPath.isDone() && runtime.seconds() < 28.7, State.DRIVING_TO_SUB_CYCLE, () -> {
                    currentPath = new SampleDriveToSubIntakePath().start();
                })
                .transition(() -> Robot.justValidSample(), State.LIFTING, () -> {
                    Robot.getInstance().cvMaster.disableSampleDetector();
                    new RetractFromVerticalIntakeCommand().schedule();
                    currentPath = new SampleLiftHighFromSubPath().start();
                })
                .state(State.PARKING)
                .onEnter(() -> logTransition(State.PARKING))
                .transition(() -> currentPath.isDone(), State.DONE)
                .state(State.DONE)
                .build();
    }

    @Override
    public void build() {
        preloadLiftingPath = new SampleHighLiftPath().build();
        cycleLiftingPath = new SampleHighLiftPath().build();

        scorePath = new SampleHighDepositPath().build();

        pathsAfterDeposit[0] = new SampleIntakeRightPath().build();
        pathsAfterDeposit[1] = new SampleIntakeCenterPath().build();
        pathsAfterDeposit[2] = new SampleIntakeLeftPath().build();
        pathsAfterDeposit[3] = new SampleParkPath().build();

        Robot.getInstance().cvMaster.stop();

        Robot.getInstance().cvMaster.startSampleStreaming();
        Robot.getInstance().cvMaster.disableSampleDetector();
    }

    @Override
    public void start() {
        scoreCount = 0;

        currentPath = preloadLiftingPath.start();

        sm.start();
        sm.setState(State.LIFTING);
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
        return Globals.mapPose(-40.5, -64, 90);
    }
}
