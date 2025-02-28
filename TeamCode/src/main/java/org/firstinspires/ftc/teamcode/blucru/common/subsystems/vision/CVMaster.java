package org.firstinspires.ftc.teamcode.blucru.common.subsystems.vision;

import android.graphics.Bitmap;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.BluSubsystem;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Config
public class CVMaster implements BluSubsystem {
    public static double fx = 913.886;
    public static double fy = 913.886;
    public static double cx = 637.951;
    public static double cy = 363.17;
    public static int GAIN = 0;
    public static long EXPOSURE = 5; // ms

    ExposureControl exposureControl;
    GainControl gainControl;

    public VisionPortal atagPortal, samplePortal;
    public AprilTagProcessor tagDetector;
    public SampleDetectionProcessor sampleDetector;

    public int numDetections;
    ArrayList<AprilTagDetection> detections;
    ArrayList<Integer> detectionIds;

    public CVMaster() {
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);

        this.tagDetector = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(fx, fy, cx, cy)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        this.sampleDetector = new SampleDetectionProcessor();

        atagPortal = new VisionPortal.Builder()
                .setCamera(Globals.hwMap.get(WebcamName.class, "webcam"))
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(tagDetector)
                .setLiveViewContainerId(viewIds[0])
                .build();
        atagPortal.setProcessorEnabled(tagDetector, false);

        while(atagPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}

        exposureControl = atagPortal.getCameraControl(ExposureControl.class);
        gainControl = atagPortal.getCameraControl(GainControl.class);

        atagPortal.stopStreaming();

        samplePortal = new VisionPortal.Builder()
                .setCamera(Globals.hwMap.get(WebcamName.class, "sample cam"))
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(sampleDetector)
                .setLiveViewContainerId(viewIds[1])
                .build();
        samplePortal.stopStreaming();
    }

    public void init() {
        read();
    }

    public void read() {
        detections = tagDetector.getDetections();

        if(atagPortal.getProcessorEnabled(tagDetector)) numDetections = detections.size();
        else numDetections = 0;
    }

    public void write() {

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Tag Portal State", atagPortal.getCameraState());
        telemetry.addData("Tag detector enabled", atagPortal.getProcessorEnabled(tagDetector));
        if(atagPortal.getProcessorEnabled(tagDetector)) {
            telemetry.addData("# tags visible: ", numDetections);

            ArrayList<Integer> ids = new ArrayList<>();
            for(AprilTagDetection detection : detections) {
                ids.add(detection.id);
            }

            telemetry.addData("Detection ids", ids);
        }

        telemetry.addData("Sample Portal State", samplePortal.getCameraState());
        telemetry.addData("Sample detector enabled", samplePortal.getProcessorEnabled(sampleDetector));
        sampleDetector.telemetry();
    }

    public void detectTag() {
        atagPortal.resumeStreaming();

        atagPortal.setProcessorEnabled(tagDetector, true);

        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(EXPOSURE, TimeUnit.MILLISECONDS);
        gainControl.setGain(GAIN);
    }

    public void startSampleStreaming() {
        samplePortal.resumeStreaming();
    }

    public void stopSampleStreaming() {
        disableSampleDetector();
        samplePortal.stopStreaming();
    }

    public void enableSampleDetector() {
        samplePortal.setProcessorEnabled(sampleDetector, true);
    }

    public void disableSampleDetector() {
        samplePortal.setProcessorEnabled(sampleDetector, false);
    }

    public void stop() {
        atagPortal.setProcessorEnabled(tagDetector, false);
        atagPortal.stopStreaming();
    }

    public boolean setExposure(double exposure) {
        return exposureControl.setExposure((long) exposure, TimeUnit.MILLISECONDS);
    }

    public boolean seesSpecimenTag() {
        for (AprilTagDetection detection: detections) {
            if(detection.id % 3 == 2) {
                return true;
            }
        }
        return false;
    }

    public boolean seesSampleTag() {
        for (AprilTagDetection detection: detections) {
            if(detection.id % 3 == 1) {
                return true;
            }
        }
        return false;
    }

    public boolean setGain(double gain) {
        return gainControl.setGain((int) gain);
    }
}