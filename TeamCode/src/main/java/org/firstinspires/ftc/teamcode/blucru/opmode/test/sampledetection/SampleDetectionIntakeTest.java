package org.firstinspires.ftc.teamcode.blucru.opmode.test.sampledetection;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.FullRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.RetractFromVerticalIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.EndEffectorRetractCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.arm.ArmPreIntakeCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampReleaseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelReverseCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.sample.SampleIntakeAtPointPath;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class SampleDetectionIntakeTest extends BluLinearOpMode {
    enum State{
        RETRACT,
        DETECTING,
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
        addWheel();
        addClamp();
        addWrist();
        addCactus();
        enableFTCDashboard();
        extension.usePivot(pivot.getMotor());
        pivot.useExtension(extension.getMotor());

        sm = new StateMachineBuilder()
                .state(State.RETRACT)
                .loop(() -> {
                    dt.teleOpDrive(gamepad1);
                    if(stickyG1.right_stick_button) dt.setHeading(Math.PI/2);

                    if(stickyG1.right_bumper) {
                        new SequentialCommandGroup(
                                new ArmPreIntakeCommand(),
                                new WaitCommand(300),
                                new ClampReleaseCommand(),
                                new WheelReverseCommand(),
                                new WaitCommand(100),
                                new EndEffectorRetractCommand()
                        ).schedule();
                    }
                })
                .transition(() -> stickyG1.b, State.DETECTING, () -> {
                    dt.pidTo(dt.pose);
                    detectTimeMillis = System.currentTimeMillis();
                })
                .state(State.DETECTING)
                .transition(() -> System.currentTimeMillis() - detectTimeMillis > 500
                    && cvMaster.sampleDetector.hasValidDetection(), State.RUNNING_PATH, () -> {
                    Pose2d blockPose = cvMaster.sampleDetector.getGlobalPose(dt.pose);
                    currentPath = new SampleIntakeAtPointPath(dt.pose.vec(), blockPose).start();
                })
                .transition(() -> stickyG1.a, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })
                .transitionTimed(0.8, State.RETRACT, () -> {
                    new FullRetractCommand().schedule();
                })
                .state(State.RUNNING_PATH)
                .transition(() -> currentPath.isDone() || Robot.justValidSample() || stickyG1.a, State.RETRACT, () -> {
                    new RetractFromVerticalIntakeCommand().schedule();
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

        if(stickyG1.y) {
            Globals.flipAlliance();
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", sm.getState());
    }
}
