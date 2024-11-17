package org.firstinspires.ftc.teamcode.blucru.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.blucru.common.path.PIDPath;
import org.firstinspires.ftc.teamcode.blucru.common.pathbase.TestPath;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(name = "PID Path test", group = "test")
public class PIDPathTest extends BluLinearOpMode {
    private enum State {
        RESETTING,
        FOLLOWING_TRAJECTORY,
        FOLLOWING_PID
    }

    PIDPath pidPath;

    double pidStartTime, pidTotalTime;

    StateMachine sm;


    @Override
    public void initialize() {
        addDrivetrain();
        enableFTCDashboard();

        Globals.setAlliance(Alliance.RED);
        pidPath = new TestPath().build();

        sm = new StateMachineBuilder()
                .state(State.RESETTING)
                .transition(() -> stickyG1.b, State.FOLLOWING_PID, () -> {
                    dt.setPoseEstimate(new Pose2d(0,0,Math.PI/2));
                    pidPath.start();
                    pidStartTime = System.currentTimeMillis();

//                    cvMaster.detectTag();
                })
                .loop(() -> {
                    dt.teleOpDrive(gamepad1);
                })
                .state(State.FOLLOWING_PID)
                .transition(() -> stickyG1.a, State.RESETTING, ()-> {
                    dt.idle();
                    pidPath.cancel();
                    dt.teleOpDrive(gamepad1);
                })
                .loop(() -> {
                    try {
                        pidPath.run();
                    } catch (Exception e) {}

                    // dt.updateAprilTags();
                })
                .build();

        sm.setState(State.RESETTING);
        sm.start();

//        cvMaster.detectTag();
    }

    @Override
    public void periodic() {
        sm.update();
        dt.drawPose();
    }

    @Override
    public void telemetry() {
        telemetry.addData("state:", sm.getState());
        telemetry.addData("pid total time: ", pidTotalTime);
        pidPath.telemetry(telemetry);
    }
}
