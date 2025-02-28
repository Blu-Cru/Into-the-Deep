package org.firstinspires.ftc.teamcode.blucru.common.subsystems.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class SampleDetectionProcessor implements VisionProcessor {
    static double[][] HOMOG_IMAGE_POINTS = {
            {945, 598}, {1168, 600},
            {931, 774}, {1203, 782}
    };
    static double[] REF_TOP_LEFT_PIXELS = {800.0, 800.0},
        REF_TOP_LEFT_INCHES = {-5.0, 20.0};
    static double PIXELS_PER_INCH = 40.0,
        RED_HUE_LOW = 150.0, RED_HUE_HIGH = 12.0,
        YELLOW_HUE_LOW = 12.0, YELLOW_HUE_HIGH = 55.0,
        BLUE_HUE_LOW = 80.0, BLUE_HUE_HIGH = 150.0,

        // calib
        fx = 1279.33, fy = 1279.33, cx = 958.363, cy = 492.062,
        // distortion
        K1 = -0.448017, K2 = 0.245668, K3 = 0.0,
        P1 = -0.000901464, P2 = 0.000996399,
        MIN_X_INCHES = -20.0, MAX_X_INCHES = 20.0,
        MIN_Y_INCHES = 9.0, MAX_Y_INCHES = 20.0;

    Mat K, DIST_COEFFS;
//        undistorted,
//        wbCorrected,
//        transformed,
//        hsv,
//        saturationThresh,
//        satMasked,
//        satEdges,
//        edges,
//        combinedEdges,
//        dilated,
//        hierarchy,
//        detectionOverlay;

    public double processingTimeMillis = 0;
    List<Pose2d> validPoses;
    int numValidPoses;

    public SampleDetectionProcessor() {
        K = new Mat(3, 3, CvType.CV_64F);
        K.put(0, 0, fx, 0, cx,
                0, fy, cy,
                0, 0, 1);

        DIST_COEFFS = new Mat(4, 1, CvType.CV_64F);
        DIST_COEFFS.put(0, 0, K1, K2, P1, P2, K3);

//        Log.i("SampleDetectionProcessor", K.toString());

//        DIST_COEFFS = new Mat(5, 1, CvType.CV_64F);
//        DIST_COEFFS.put(0, 0, new double[] {K1, K2, P1, P2, K3});
        validPoses = new ArrayList<>();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Mat processFrame(Mat frame, long captureTimeNanos) {
        double startNanoTime = System.nanoTime();
        validPoses = new ArrayList<>();

//        Mat newCameraMatrix = Calib3d.getOptimalNewCameraMatrix(K, DIST_COEFFS, frame.size(), 1, frame.size());
//
//        Mat undistorted = new Mat();
//        Calib3d.undistort(frame, undistorted, K, DIST_COEFFS, newCameraMatrix);
//        newCameraMatrix.release();

        Mat undistorted = undistort(frame);

//        Mat wbCorrected = applyGrayWorldWhiteBalance(undistorted);
//
//        Mat transformed = doHomographyTransform(wbCorrected);
//
////        Imgproc.resize(transformed, transformed, new Size(960, 520));
//
//        Mat hsv = new Mat();
//        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);
////
//        Mat hueChannel = new Mat();
//        Mat satChannel = new Mat();
//        Core.extractChannel(hsv, hueChannel, 0);
//        Core.extractChannel(hsv, satChannel, 1);
//
//        Mat saturationThresh = new Mat();
//        Core.inRange(hsv, new Scalar(0, 45, 0), new Scalar(255, 255, 255), saturationThresh);
//
//        Mat satMasked = new Mat();
//        Core.bitwise_and(transformed, transformed, satMasked, saturationThresh);
//
//        Mat satEdges = new Mat();
//        Imgproc.Canny(satMasked, satEdges, 100, 200);
//
//        Mat edges = new Mat();
//        Imgproc.Canny(transformed, edges, 40, 100);
//
//        Mat combinedEdges = new Mat();
//        Core.bitwise_or(edges, satEdges, combinedEdges);
//
//        Mat dilationElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2.5, 2.5));
//        Mat dilated = new Mat();
//        Imgproc.dilate(combinedEdges, dilated, dilationElement);
//
//        List<MatOfPoint> contours = new ArrayList<>();
//        Mat hierarchy = new Mat();
//        Imgproc.findContours(dilated, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//
////        RotatedRect[] rects = new RotatedRect[contours.size()];
////        double[] normalizedAngles = new double[contours.size()];
//
//        Mat detectionOverlay = transformed.clone();
//        List<MatOfPoint> validRects = new ArrayList<>();
//
//        for(int i = 0; i < contours.size(); i++) {
//            MatOfPoint cnt = contours.get(i);
//
//            double area = Imgproc.contourArea(cnt);
//            if(area < 2000.0 || area > 14000.0) {
//                Log.d("SampleDetectionProcessor", "Contour discarded with area:" + area);
//                continue;
//            }
//
//            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(cnt.toArray()));
////            rects[i] = rect;
//            double normalizedAngle = rect.size.height > rect.size.width ? 90.0 - rect.angle : -rect.angle;
////            normalizedAngles[i] = rect.size.height > rect.size.width ? 90.0 - rect.angle : -rect.angle;
//
//            if(rect.size.width == 0 || rect.size.height == 0) continue;
//            double ratio = Math.max(rect.size.width, rect.size.height) / Math.min(rect.size.width, rect.size.height);
//            if(ratio < 2.0 || ratio > 3.0) {
//                Log.d("SampleDetectionProcessor", "Contour discarded with ratio:" + ratio);
//                continue;
//            }
//
////            List<Mat> channels = new ArrayList<>();
////            Core.split(hsv, channels);
//
//            Point[] boxPoints = new Point[4];
//            rect.points(boxPoints);
//            MatOfPoint matOfRectPoints = new MatOfPoint(boxPoints);
//
//            Mat rotatedRectMask = Mat.zeros(transformed.size(), CvType.CV_8UC1);
//            Imgproc.fillConvexPoly(rotatedRectMask, matOfRectPoints, new Scalar(255));
//
//            Scalar meanHue = Core.mean(hueChannel, rotatedRectMask);
//            Scalar meanSat = Core.mean(satChannel, rotatedRectMask);
//
//            if(meanSat.val[0] < 45.0) {
//                Log.d("SampleDetectionProcessor", "Contour discarded with sat: " + meanSat.val[0]);
//                continue;
//            }
//
//            if(Globals.alliance == Alliance.RED) {
//                if (BLUE_HUE_LOW < meanHue.val[0] && meanHue.val[0] < BLUE_HUE_HIGH){
//                    Log.d("SampleDetectionProcessor", "Contour discarded with hue: " + meanHue.val[0]);
//                    continue;
//                }
//            } else {
//                if((meanHue.val[0] > RED_HUE_LOW || meanHue.val[0] < RED_HUE_HIGH)) {
//                    Log.d("SampleDetectionProcessor", "Contour discarded with hue: " + meanHue.val[0]);
//                    continue;
//                }
//            }
//
//            Pose2d blockPose = new Pose2d(getPoint(rect.center), Math.toRadians(normalizedAngle));
//
//            if(blockPose.getX() < MIN_X_INCHES || blockPose.getX() > MAX_X_INCHES
//                    || blockPose.getY() < MIN_Y_INCHES || blockPose.getY() > MAX_Y_INCHES) {
//                Log.d("SampleDetectionProcessor", "Contour discarded with position: " + blockPose);
//                continue;
//            }
//
//            validRects.add(matOfRectPoints);
//            validPoses.add(new Pose2d(getPoint(rect.center), Math.toRadians(normalizedAngle)));
//
//            cnt.release();
//            matOfRectPoints.release();
//            rotatedRectMask.release();
//        }
//
//        Imgproc.drawContours(detectionOverlay, validRects, -1, new Scalar(0, 255, 0), 2);
//
//        numValidPoses = validPoses.size();
//        // TODO: loop through detections, score them based on location, orientation
//        // find pose of highest score

        undistorted.copyTo(frame);

        undistorted.release();
//        wbCorrected.release();
//        transformed.release();
//        hsv.release();
//        saturationThresh.release();
//        satMasked.release();
//        satEdges.release();
//        edges.release();
//        combinedEdges.release();
//        dilated.release();
//        hierarchy.release();
////        detectionOverlay.release();
//

        processingTimeMillis = (System.nanoTime() - startNanoTime) / 1e6;
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }

    public Mat undistort(Mat frame) {
        Mat newCameraMatrix = Calib3d.getOptimalNewCameraMatrix(
                K,
                DIST_COEFFS,
                frame.size(),
                0,
                frame.size());

        Mat map1 = new Mat(), map2 = new Mat();
        Calib3d.initUndistortRectifyMap(K, DIST_COEFFS, new Mat(), newCameraMatrix, frame.size(), CvType.CV_16SC2, map1, map2);
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

    public Mat applyGrayWorldWhiteBalance(Mat src) {
        // Ensure the image is in 3-channel BGR format
//        if (src.channels() != 3) {
//            throw new IllegalArgumentException("Input image must have 3 channels (BGR)");
//        }

        // Compute the mean of each channel (B, G, R)
        Scalar mean = Core.mean(src);

        double meanB = mean.val[0];  // Blue channel mean
        double meanG = mean.val[1];  // Green channel mean
        double meanR = mean.val[2];  // Red channel mean

        // Calculate scaling factors for each channel
        double meanGray = (meanB + meanG + meanR) / 3.0;
        double scaleB = meanGray / meanB;
        double scaleG = meanGray / meanG;
        double scaleR = meanGray / meanR;

        // Apply scaling factors
        Mat balanced = new Mat();
        src.convertTo(balanced, CvType.CV_32F); // Convert to float for multiplication

        Core.multiply(balanced, new Scalar(scaleB, scaleG, scaleR), balanced);

        balanced.convertTo(balanced, CvType.CV_8U); // Convert back to 8-bit

        return balanced;
    }

    public Mat doHomographyTransform(Mat src) {
        Mat M = Imgproc.getPerspectiveTransform(
                new MatOfPoint2f(
                        new Point(HOMOG_IMAGE_POINTS[0][0], HOMOG_IMAGE_POINTS[0][1]), new Point(HOMOG_IMAGE_POINTS[1][0], HOMOG_IMAGE_POINTS[1][1]),
                        new Point(HOMOG_IMAGE_POINTS[2][0], HOMOG_IMAGE_POINTS[2][1]), new Point(HOMOG_IMAGE_POINTS[3][0], HOMOG_IMAGE_POINTS[3][1])
                ),
                new MatOfPoint2f(
                        new Point(REF_TOP_LEFT_PIXELS[0], REF_TOP_LEFT_PIXELS[1]), new Point(REF_TOP_LEFT_PIXELS[0] + PIXELS_PER_INCH * 5.0, REF_TOP_LEFT_PIXELS[1]),
                        new Point(REF_TOP_LEFT_PIXELS[0], REF_TOP_LEFT_PIXELS[1] + PIXELS_PER_INCH * 5.0), new Point(REF_TOP_LEFT_PIXELS[0] + PIXELS_PER_INCH * 5.0, REF_TOP_LEFT_PIXELS[1] + PIXELS_PER_INCH * 5.0)
                )
        );

        Mat transformed = new Mat();
        Imgproc.warpPerspective(src, transformed, M, src.size());

        return transformed;
    }

    private Vector2d getPoint(Point centerPixels) {
        double refOffsetX = centerPixels.x-REF_TOP_LEFT_PIXELS[0];
        double refOffsetY = -(centerPixels.y-REF_TOP_LEFT_PIXELS[1]);

        double inchOffsetX = REF_TOP_LEFT_INCHES[0]+refOffsetX/PIXELS_PER_INCH;
        double inchOffsetY = REF_TOP_LEFT_INCHES[1]+refOffsetY/PIXELS_PER_INCH;
        return new Vector2d(inchOffsetX, inchOffsetY);
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
        tele.addData("Num. of Sample Detections", validPoses.size());
        for(Pose2d pose : validPoses) {
            tele.addData("Detected valid pose: ", pose);
        }
        tele.addData("Processing time (ms)", processingTimeMillis);
    }
}
