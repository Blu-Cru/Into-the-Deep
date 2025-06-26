package org.firstinspires.ftc.teamcode.blucru.opmode.test.sample_detection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@TeleOp(group = "3")
public class SampleDetectionTest extends BluLinearOpMode {
    enum State {
        IDLE,
        DETECTING
    }

    StateMachine sm;

    @Override
    public void initialize() {
        addCVMaster();
        enableFTCDashboard();

        cvMaster.startSampleStreaming();
        FtcDashboard.getInstance().startCameraStream((CameraStreamSource) cvMaster.samplePortal, 30);


        sm = new StateMachineBuilder()
                .state(State.IDLE)
                .transition(() -> stickyG1.a, State.DETECTING, () -> {
                    cvMaster.startSampleStreaming();
                    cvMaster.enableSampleDetector();
                    FtcDashboard.getInstance().startCameraStream((CameraStreamSource) cvMaster.samplePortal, 30);
                })
                .state(State.DETECTING)
                .transition(() -> stickyG1.a, State.IDLE, () -> {
                    cvMaster.stopSampleStreaming();
                    FtcDashboard.getInstance().stopCameraStream();
                })
                .build();

        sm.setState(State.IDLE);
        sm.start();
    }

    @Override
    public void initLoop() {
        sm.update();

        if(stickyG1.b) {
            Globals.flipAlliance();
        }

        cvMaster.telemetry(telemetry);
        telemetry();
    }

    @Override
    public void telemetry() {
        telemetry.addData("State", sm.getState());
        telemetry.addData("Alliance", Globals.alliance);
    }
}
