package org.firstinspires.ftc.teamcode.blucru.opmode.test.sample_detection;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.command_base.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.command_base.intake.SpitCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.path_base.sample.SampleIntakeAtPointPath;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class SampleDetectionIntakeTest extends BluLinearOpMode {
    enum State{
        RETRACT,
        DETECTING_SAMPLE,
        DETECTING_SPEC,
        RUNNING_PATH
    }

    StateMachine sm;
    double detectTimeMillis;
    Path currentPath;

    @Override
    public void initialize() {
        addCVMaster();
        addDrivetrain();
        addExtension();
        addPivot();
        addArm();
        addSpinWrist();
        addUpDownWrist();
        addTurret();
        addClaw();
        addCactus();
        enableFTCDashboard();
        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());

        sm = new StateMachineBuilder()
                .state(State.RETRACT)
                .transition(() -> stickyG1.b, State.DETECTING_SAMPLE, () -> {
                    dt.pidTo(dt.pose);
                    detectTimeMillis = System.currentTimeMillis();
                })
                .transition(() -> stickyG1.y, State.DETECTING_SPEC, () -> {
                    dt.pidTo(dt.pose);
                    detectTimeMillis = System.currentTimeMillis();
                })
                .loop(() -> {
                    dt.teleOpDrive(gamepad1);
                    if(stickyG1.right_stick_button) dt.setHeading(Math.PI/2);

                    if(stickyG1.right_bumper) {
                        new SpitCommand().schedule();
                    }

                    if(stickyG1.x) {
                        Globals.flipAlliance();
                    }
                })

                .state(State.DETECTING_SAMPLE)
                .transition(() -> System.currentTimeMillis() - detectTimeMillis > 500
                    && cvMaster.sampleDetector.detectionList.hasSample(), State.RUNNING_PATH, () -> {
                    currentPath = new SampleIntakeAtPointPath(dt.pose, cvMaster.sampleDetector.detectionList.getBestSamplePose()).start();
                })
                .transition(() -> stickyG1.a, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })
                .transitionTimed(0.8, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })

                .state(State.DETECTING_SPEC)
                .transition(() -> System.currentTimeMillis() - detectTimeMillis > 500
                        && cvMaster.sampleDetector.detectionList.hasSpec(), State.RUNNING_PATH, () -> {
                    currentPath = new SampleIntakeAtPointPath(dt.pose, cvMaster.sampleDetector.detectionList.getBestSpecPose()).start();
                })
                .transition(() -> stickyG1.a, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })
                .transitionTimed(0.8, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })

                .state(State.RUNNING_PATH)
                .transition(() -> currentPath.isDone() || stickyG1.a, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })
                .loop(() -> {
                    currentPath.run();
                })
                .build();
    }

    @Override
    public void onStart() {
        cvMaster.startSampleStreaming();
        cvMaster.enableSampleDetector();
        sm.setState(State.RETRACT);
        sm.start();
    }

    @Override
    public void periodic() {
        sm.update();
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", sm.getState());
    }
}
