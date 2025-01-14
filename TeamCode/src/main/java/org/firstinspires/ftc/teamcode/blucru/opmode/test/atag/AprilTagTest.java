package org.firstinspires.ftc.teamcode.blucru.opmode.test.atag;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.vision.AprilTagPoseGetter;
import org.firstinspires.ftc.teamcode.blucru.opmode.BluLinearOpMode;

@Config
@TeleOp(name="April Tag Test", group="test")
public class AprilTagTest extends BluLinearOpMode {
    private enum State {
        IDLE,
        DETECTING
    }
    public static double EXPOSURE_MS = 50;
    public static double GAIN = 10;

    State state = State.IDLE;

    @Override
    public void initialize() {
        addDrivetrain();
        addCVMaster();
        enableFTCDashboard();
    }

    public void initLoop() {
        if (stickyG1.a) {
            cvMaster.detectTag();
            state = State.DETECTING;
            FtcDashboard.getInstance().startCameraStream((CameraStreamSource) cvMaster.visionPortal, 30);
        }

        if (stickyG1.x) {
            cvMaster.stop();
            state = State.IDLE;
            FtcDashboard.getInstance().stopCameraStream();
        }

        cvMaster.setExposure(EXPOSURE_MS);
        cvMaster.setGain(GAIN);

        try {
            dt.setPoseEstimate(AprilTagPoseGetter.getRobotPoseAtTimeOfFrame(cvMaster.tagDetector.getDetections()));
        } catch (Exception e) {

        }
        robot.telemetry(telemetry);
        dt.drawPose();
    }

    public void telemetry() {
        telemetry.addData("state:", state);
    }
}
