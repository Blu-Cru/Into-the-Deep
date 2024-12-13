package org.firstinspires.ftc.teamcode.blucru.opmode.test.atag;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "AprilTag Drive Test", group = "test")
public class AprilTagDriveTest extends BluLinearOpMode {
    enum State {
        DRIVER_CONTROL,
        DRIVE_TO_DEPOSIT,
    }
    State state = State.DRIVER_CONTROL;
    StateMachine sm;

    @Override
    public void initialize() {
        addDrivetrain();
        addCVMaster();
        enableFTCDashboard();

        sm = new StateMachineBuilder()
                .state(State.DRIVER_CONTROL)
                .transition(() -> stickyG1.a, State.DRIVE_TO_DEPOSIT, () -> {
                    dt.pidTo(new Pose2d(-53, -53, Math.PI/4));
                })
                .loop(() -> {
                    dt.teleOpDrive(gamepad1);

                    if(stickyG1.right_stick_button) {
                        dt.setHeading(Math.PI/2);
                    }
                })
                .state(State.DRIVE_TO_DEPOSIT)
                .transition(() -> stickyG1.a, State.DRIVER_CONTROL, () -> {
                    dt.idle();
                })
                .build();

        sm.start();
    }

    @Override
    public void onStart() {
        cvMaster.detectTag();
        sm.setState(State.DRIVER_CONTROL);
    }

    @Override
    public void periodic() {
        sm.update();
        dt.updateAprilTags(cvMaster.tagDetector);
        dt.drawPose();
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", state);
    }
}
