package org.firstinspires.ftc.teamcode.blucru.opmode.test.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.clamp.ClampGrabCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.endeffector.wheel.WheelStopCommand;
import org.firstinspires.ftc.teamcode.blucru.common.commandbase.spline.BoxtubeSplineCommand;
import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPathBuilder;
import org.firstinspires.ftc.teamcode.blucru.common.path.Path;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "test")
public class TeleDrivePIDTest extends BluLinearOpMode {
    enum State{
        IDLE,
        SCANNING,
        CYCLE_INTAKE,
        CYCLE_DEPO_PATH,
        CYCLE_DEPO_MANUAL
    }

    Path currentPath;
    Path cycleIntakePath, cycleDepoPath;

    StateMachine sm;

    @Override
    public void initialize() {
        enableFTCDashboard();
        addDrivetrain();
        addCVMaster();

        cycleIntakePath = new PIDPathBuilder().setPower(0.7)
                .addMappedPoint(24, -48.5, -60, 7)
                .waitMillis(400)
                .setPower(0.25)
                .addMappedPoint(29, -53, -60)
                .waitMillis(200)
                .build();

        cycleDepoPath = new PIDPathBuilder().setPower(0.8)
                .schedule(new SequentialCommandGroup(
                        new WheelStopCommand(),
                        new ClampGrabCommand(),
                        new BoxtubeSplineCommand(
                                new Vector2d(20,42),
                                new Pose2d(-8.6, 30, Math.PI),
                                0,
                                0.95
                        )
                ))
                .addMappedPoint(7, -40, 270, 5)
                .build();
        currentPath = cycleIntakePath;

        sm = new StateMachineBuilder()
                .state(State.IDLE)
                .onEnter(() -> currentPath.cancel())
                .loop(() -> {
                    dt.teleOpDrive(gamepad1);
//                    dt.updateAprilTags(cvMaster.tagDetector);
                })
                .transition(() -> stickyG1.b && cvMaster.numDetections > 0, State.CYCLE_INTAKE, () -> {
                    dt.updateAprilTags(cvMaster.tagDetector);
                    currentPath = cycleIntakePath.start();
                })

                .state(State.CYCLE_INTAKE)
                .transition(() -> stickyG1.a, State.IDLE)
                .transition(() -> currentPath.isDone(), State.CYCLE_DEPO_PATH, () -> currentPath = cycleDepoPath.start())
                .loop(() -> {
                    currentPath.run();
//                    dt.updateAprilTags(cvMaster.tagDetector);
                })

                .state(State.CYCLE_DEPO_PATH)
                .transition(() -> stickyG1.a, State.IDLE)
                .transition(() -> currentPath.isDone(), State.CYCLE_DEPO_MANUAL, () -> {
                    dt.setDrivePower(0.7);
                    dt.idle();
                })
                .loop(() -> currentPath.run())

                .state(State.CYCLE_DEPO_MANUAL)
                .transition(() -> stickyG1.a, State.IDLE)
                .transition(() -> stickyG1.b, State.CYCLE_INTAKE, () -> currentPath = cycleIntakePath.start())
                .loop(() -> dt.pidYHeadingMapped(gamepad1.left_stick_x, -34, -Math.PI/2))

//                .state(State.SCANNING)
//                .onEnter(() -> {
//                    dt.pidTo(dt.getStopPose());
//                })
//                .transitionTimed(0.5, () -> cvMaster.numDetections > 3)
                .build();
    }

    @Override
    public void onStart() {
        sm.start();
        sm.setState(State.IDLE);
        cvMaster.detectTag();
    }

    @Override
    public void periodic() {
        sm.update();
        dt.drawPose();

        if(stickyG1.right_stick_button) {
            dt.setHeading(Math.PI/2);
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", sm.getState());
    }
}
