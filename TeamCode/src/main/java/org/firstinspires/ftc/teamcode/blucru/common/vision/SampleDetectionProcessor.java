package org.firstinspires.ftc.teamcode.blucru.common.vision;

import android.graphics.Canvas;
import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.Robot;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.teamcode.blucru.common.util.Sample;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Config
public class SampleDetectionProcessor implements VisionProcessor {
    static double[][] HOMOG_IMAGE_POINTS = {
            {853, 612}, {1201, 612},
            {812, 947}, {1287, 941}
    };
    static double DIST_BETWEEN_POINTS = 5.6;
    static double[] REF_TOP_LEFT_PIXELS = {800.0/3, 800.0/3},
        REF_TOP_LEFT_INCHES = {-5.6, 18.2};
    public static double PIXELS_PER_INCH = 20.0,
        MIN_SAT_MASK,
        RED_HUE_LOW = 110.0, RED_HUE_HIGH = 150.0,
        YELLOW_HUE_LOW = 12.0, YELLOW_HUE_HIGH = 55.0,
        BLUE_HUE_LOW = 0.0, BLUE_HUE_HIGH = 35.0,

        // calib
        fx = 1279.33, fy = 1279.33, cx = 958.363, cy = 492.062,
        // distortion
        K1 = -0.448017, K2 = 0.245668, K3 = 0.0,
        P1 = -0.000901464, P2 = 0.000996399,
        MAX_DETECTION_DISTANCE = 12, MIN_DETECTION_X = 13.0,
        MAX_DETECTION_DELTA_Y = 6.0;
    static Vector2d OPTIMAL_POINT = new Vector2d(19, 6.0);
    public static double RATIO_MIN = 1.7, RATIO_MAX = 2.6;
    public static double AREA_MIN = 1000, AREA_MAX = 1650;
    public static double MIN_SAT = 70;
    Mat K, DIST_COEFFS,
        map1, map2,
        homogM, dilationElement;

    double processingTimeMillis = 0;

    public SampleDetectionsList detectionList;

    public SampleDetectionProcessor() {
        K = new Mat(3, 3, CvType.CV_64F);
        K.put(0, 0, fx, 0, cx,
                0, fy, cy,
                0, 0, 1);

        DIST_COEFFS = new Mat(4, 1, CvType.CV_64F);
        DIST_COEFFS.put(0, 0, K1, K2, P1, P2, K3);
        detectionList = new SampleDetectionsList();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        Size size = new Size(width, height);
        Mat newCameraMatrix = Calib3d.getOptimalNewCameraMatrix(
                K,
                DIST_COEFFS,
                size,
                1,
                size);

        map1 = new Mat();
        map2 = new Mat();
        Calib3d.initUndistortRectifyMap(K, DIST_COEFFS, new Mat(), newCameraMatrix, size, CvType.CV_16SC2, map1, map2);

        homogM = Imgproc.getPerspectiveTransform(
                new MatOfPoint2f(
                        new Point(HOMOG_IMAGE_POINTS[0][0], HOMOG_IMAGE_POINTS[0][1]), new Point(HOMOG_IMAGE_POINTS[1][0], HOMOG_IMAGE_POINTS[1][1]),
                        new Point(HOMOG_IMAGE_POINTS[2][0], HOMOG_IMAGE_POINTS[2][1]), new Point(HOMOG_IMAGE_POINTS[3][0], HOMOG_IMAGE_POINTS[3][1])
                ),
                new MatOfPoint2f(
                        new Point(REF_TOP_LEFT_PIXELS[0], REF_TOP_LEFT_PIXELS[1]), new Point(REF_TOP_LEFT_PIXELS[0] + PIXELS_PER_INCH * DIST_BETWEEN_POINTS, REF_TOP_LEFT_PIXELS[1]),
                        new Point(REF_TOP_LEFT_PIXELS[0], REF_TOP_LEFT_PIXELS[1] + PIXELS_PER_INCH * DIST_BETWEEN_POINTS), new Point(REF_TOP_LEFT_PIXELS[0] + PIXELS_PER_INCH * DIST_BETWEEN_POINTS, REF_TOP_LEFT_PIXELS[1] + PIXELS_PER_INCH * DIST_BETWEEN_POINTS)
                )
        );

        dilationElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    }

    @Override
    public Mat processFrame(Mat frame, long captureTimeNanos) {
        // calculate global pose, and add global pose to sorted poses
        SampleDetectionsList tempList = new SampleDetectionsList();
        Pose2d drivePose;
        try {
            drivePose = Robot.getInstance().dt.pose;
        } catch (Exception e) {
            return null;
        }
        double startNanoTime = System.nanoTime();

        Mat undistorted = undistort(frame);

        Mat transformed = doHomographyTransform(undistorted);

        Mat wbCorrected = applyGrayWorldAWB(transformed);

        Mat hsv = new Mat();
        Imgproc.cvtColor(wbCorrected, hsv, Imgproc.COLOR_BGR2HSV);

        List<Mat> channels = new ArrayList<>(3);
        Core.split(hsv, channels);

// Now you can access individual channels
        Mat hueChannel = channels.get(0);
        Mat satChannel = channels.get(1);

        Mat saturationThresh = new Mat();
        Core.inRange(satChannel, new Scalar(45), new Scalar(255), saturationThresh);

        Mat satMasked = new Mat();
        Core.bitwise_and(wbCorrected, wbCorrected, satMasked, saturationThresh);

        Mat satEdges = new Mat();
        Imgproc.Canny(satMasked, satEdges, 50, 200);

        Mat edges = new Mat();
        Imgproc.Canny(wbCorrected, edges, 0, 150);

        Mat combinedEdges = new Mat();
        Core.bitwise_or(edges, satEdges, combinedEdges);

        Mat dilated = new Mat();
        Imgproc.dilate(combinedEdges, dilated, dilationElement);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(dilated, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        Mat detectionOverlay = wbCorrected.clone();
        List<MatOfPoint> validRects = new ArrayList<>();

        Scalar meanHue, meanSat;

        double minDistance = Double.POSITIVE_INFINITY;
        Pose2d tempBestPose = new Pose2d();

        for(int i = 0; i < contours.size(); i++) {

            MatOfPoint cnt = contours.get(i);

            double area = Imgproc.contourArea(cnt);
            if(area < AREA_MIN || area > AREA_MAX) {
                Log.d("SampleDetectionProcessor", "Contour discarded with area:" + area);
                continue;
            }

            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(cnt.toArray()));

            Vector2d point = getRobotPoint(rect.center);

            // discard sample out of range
            if (Math.abs(point.getY()) > MAX_DETECTION_DELTA_Y) {
                Log.d("SampleDetectionProcessor", "Rect discarded, Y out of range");
                continue;
            }

            double distance = point.minus(OPTIMAL_POINT).norm();

            if(distance > MAX_DETECTION_DISTANCE) {
                Log.d("SampleDetectionProcessor", "Contour discarded with distance: " + distance);
                continue;
            }
            if(point.getX() < MIN_DETECTION_X) {
                Log.d("SampleDetectionProcessor", "Contour discarded with x: " + point.getX());
                continue;
            }

            if(rect.size.width == 0 || rect.size.height == 0) continue;
            double ratio = Math.max(rect.size.width, rect.size.height) / Math.min(rect.size.width, rect.size.height);
            if(ratio < RATIO_MIN || ratio > RATIO_MAX) {
                Log.d("SampleDetectionProcessor", "Contour discarded with ratio:" + ratio);
                continue;
            }

            Point[] boxPoints = new Point[4];
            rect.points(boxPoints);
            MatOfPoint matOfRectPoints = new MatOfPoint(boxPoints);

            Mat rotatedRectMask = Mat.zeros(wbCorrected.size(), CvType.CV_8UC1);
            Imgproc.fillConvexPoly(rotatedRectMask, matOfRectPoints, new Scalar(255));

            meanSat = Core.mean(satChannel, rotatedRectMask);
            if(meanSat.val[0] < MIN_SAT) {
                Log.d("SampleDetectionProcessor", "Contour discarded with sat: " + meanSat.val[0]);
                continue;
            }

            meanHue = Core.mean(hueChannel, rotatedRectMask);

            Sample blockColor = getSample(meanHue);

            double normalizedAngle;
            if (rect.size.width < rect.size.height) normalizedAngle = - rect.angle;
            else normalizedAngle = -rect.angle - 90.0;

            Pose2d pose = new Pose2d(point, Angle.norm(Math.toRadians(normalizedAngle)));

            tempList.add(new SampleDetection(getGlobalPose(pose, drivePose), blockColor, distance, Angle.norm(Math.toRadians(normalizedAngle))));

            // print saturation
            Imgproc.putText(detectionOverlay, "Sat: " + meanSat, rect.center, Imgproc.FONT_HERSHEY_COMPLEX, 0.6, new Scalar(0, 255, 0), 2);
            // print hue
//            Imgproc.putText(detectionOverlay, "Hue: " + meanHue, rect.center, Imgproc.FONT_HERSHEY_COMPLEX, 0.6, new Scalar(0, 255, 0), 2);
            // print area
//            Imgproc.putText(detectionOverlay, "Area: " + area, rect.center, Imgproc.FONT_HERSHEY_COMPLEX, 0.6, new Scalar(0, 255, 0), 2);
//             print ratio
//            Imgproc.putText(detectionOverlay, "Ratio: " + ratio, rect.center, Imgproc.FONT_HERSHEY_COMPLEX, 0.6, new Scalar(0, 255, 0), 2);
            // print center
//            Imgproc.putText(detectionOverlay, "(" + Math.round(point.getX()*100)/100.0 + ", " + Math.round(point.getY() *100) / 100.0+ ")", rect.center, Imgproc.FONT_HERSHEY_COMPLEX, 0.6, new Scalar(0, 255, 0), 2);

            validRects.add(matOfRectPoints);

            cnt.release();
//            matOfRectPoints.release();
            rotatedRectMask.release();
        }

        Imgproc.drawContours(detectionOverlay, validRects, -1, new Scalar(0, 255, 0), 2);

        // saves temp list into detectionList
        detectionList = tempList;

        Mat output = new Mat();
        Imgproc.resize(detectionOverlay, output, frame.size());
        output.copyTo(frame);
        output.release();

//        transformed.copyTo(frame);

        undistorted.release();
//        wbCorrected.release();
        wbCorrected.release();
        hueChannel.release();
        satChannel.release();
        hsv.release();
        saturationThresh.release();
        satMasked.release();
        satEdges.release();
        edges.release();
        combinedEdges.release();
        dilated.release();
        hierarchy.release();
        detectionOverlay.release();

        processingTimeMillis = (System.nanoTime() - startNanoTime) / 1e6;
        return null;
    }

    @NonNull
    private static Sample getSample(Scalar meanHue) {
        if (BLUE_HUE_LOW < meanHue.val[0] && meanHue.val[0] < BLUE_HUE_HIGH){
            return Sample.BLUE;
//                Log.d("SampleDetectionProcessor", "Contour discarded with hue: " + (int) meanHue.val[0]);
        }
        if(meanHue.val[0] > RED_HUE_LOW && meanHue.val[0] < RED_HUE_HIGH) {
            return Sample.RED;
//                Log.d("SampleDetectionProcessor", "Contour discarded with hue: " + meanHue.val[0]);
        }
        return Sample.YELLOW;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    public Mat undistort(Mat frame) {
        Mat undistorted = new Mat();
        Imgproc.remap(frame, undistorted, map1, map2, Imgproc.INTER_LINEAR);
//
////        Log.i("SampleDetectionProcessor", newCameraMatrix.toString());
//
//        Mat undistorted = new Mat();
////        Calib3d.undistort(frame, undistorted, K, DIST_COEFFS);
//        Calib3d.undistort(frame, undistorted, K, DIST_COEFFS, newCameraMatrix);
        return undistorted;
    }

    public Mat doHomographyTransform(Mat src) {
        Mat transformed = new Mat();
        Imgproc.warpPerspective(src, transformed, homogM, new Size(640, 360));

        return transformed;
    }

    // get position of block
    private Vector2d getRobotPoint(Point centerPixels) {
        double refOffsetX = centerPixels.x- REF_TOP_LEFT_PIXELS[0];
        double refOffsetY = -(centerPixels.y- REF_TOP_LEFT_PIXELS[1]);

        double inchOffsetX = REF_TOP_LEFT_INCHES[0]+refOffsetX/PIXELS_PER_INCH;
        double inchOffsetY = REF_TOP_LEFT_INCHES[1]+refOffsetY/PIXELS_PER_INCH;
        return new Vector2d(inchOffsetX, inchOffsetY).rotated(Math.toRadians(-90));
    }

    private double[] getMeanHueSat(Mat hsv, RotatedRect rect) {
        Mat roi = new Mat(hsv, rect.boundingRect());

        List<Mat> channels = new ArrayList<>();
        Core.split(hsv, channels);

        Mat hue = channels.get(0);
        Mat sat = channels.get(1);

        Scalar meanHue = Core.mean(hue);
        Scalar meanSat = Core.mean(sat);

        return new double[] {meanHue.val[0], meanSat.val[0]};
    }

    public void telemetry() {
        Telemetry tele = Globals.tele;
        tele.addLine("Sample Detection Processor:");
        tele.addData("Num. of Sample Detections", detectionList.size());
        detectionList.telemetry();
        tele.addData("Processing time (ms)", processingTimeMillis);
    }

    public Mat applyGrayWorldAWB(Mat src) {
        List<Mat> channels = new ArrayList<>();
        Core.split(src, channels);

        Scalar avgR = Core.mean(channels.get(2)); // Red channel
        Scalar avgG = Core.mean(channels.get(1)); // Green channel
        Scalar avgB = Core.mean(channels.get(0)); // Blue channel

        double avgGray = (avgR.val[0] + avgG.val[0] + avgB.val[0]) / 3.0;

        channels.get(2).convertTo(channels.get(2), -1, avgGray / avgR.val[0]); // Normalize Red
        channels.get(1).convertTo(channels.get(1), -1, avgGray / avgG.val[0]); // Normalize Green
        channels.get(0).convertTo(channels.get(0), -1, avgGray / avgB.val[0]); // Normalize Blue

        Mat balanced = new Mat();
        Core.merge(channels, balanced);
        return balanced;
    }

    public boolean hasValidDetection() {
        return !detectionList.isEmpty();
    }

    public Pose2d getGlobalPose(Pose2d blockPose, Pose2d drivePose){
        Vector2d vec = drivePose.vec().plus(blockPose.vec().rotated(drivePose.getHeading()));

        double heading = drivePose.getHeading() + blockPose.getHeading();

        return new Pose2d(vec, heading);
    }
}
