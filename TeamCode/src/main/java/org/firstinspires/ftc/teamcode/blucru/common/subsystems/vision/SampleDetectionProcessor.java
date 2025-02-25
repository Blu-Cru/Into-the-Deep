package org.firstinspires.ftc.teamcode.blucru.common.subsystems.vision;

import android.graphics.Canvas;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
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

public class SampleDetectionProcessor implements VisionProcessor {
    static double[][] HOMOG_IMAGE_POINTS = {
            {895.0, 607.0}, {1155.0, 602.0},
            {870.0, 810.0}, {1207, 805.0}
    };
    static double[] REF_TOP_LEFT_PIXELS = {400.0, 400.0},
        REF_TOP_LEFT_INCHES = {0.0, 0.0};
    static double PIXELS_PER_INCH = 40.0,
        RED_HUE_LOW = 150.0, RED_HUE_HIGH = 12.0,
        YELLOW_HUE_LOW = 12.0, YELLOW_HUE_HIGH = 55.0,
        BLUE_HUE_LOW = 80.0, BLUE_HUE_HIGH = 150.0,

        // calib
        fx = 1279.33, fy = 1279.33, cx = 958.363, cy = 492.062,
        // distortion
        K1 = -0.448017, K2 = 0.245668, K3 = 0.0,
        P1 = -0.000901464, P2 = 0.000996399;

    Mat K, DIST_COEFFS;
    public double processingTimeMillis = 0;

    public SampleDetectionProcessor() {
        K = new Mat(3, 3, CvType.CV_64F);
        K.put(0, 0, new double[] {fx, 0, cx, 0, fy, cy, 0, 0, 1});

        DIST_COEFFS = new Mat(5, 1, CvType.CV_64F);
        DIST_COEFFS.put(0, 0, new double[] {K1, K2, P1, P2, K3});
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        double startNanoTime = System.nanoTime();

        Mat undistorted = undistort(frame);

        Mat wbCorrected = applyGrayWorldWhiteBalance(undistorted);

        Mat transformed = doHomographyTransform(wbCorrected);

//        Imgproc.resize(transformed, transformed, new Size(960, 520));

        Mat hsv = new Mat();
        Imgproc.cvtColor(transformed, hsv, Imgproc.COLOR_BGR2HSV);

        Mat saturationThresh = new Mat();
        Core.inRange(hsv, new Scalar(0, 45, 0), new Scalar(255, 255, 255), saturationThresh);

        Mat satMasked = new Mat();
        Core.bitwise_and(transformed, transformed, satMasked, saturationThresh);

        Mat satEdges = new Mat();
        Imgproc.Canny(satMasked, satEdges, 100, 200);

        Mat edges = new Mat();
        Imgproc.Canny(transformed, edges, 40, 100);

        Mat combinedEdges = new Mat();
        Core.bitwise_or(edges, satEdges, combinedEdges);

        Mat dilationElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2.5, 2.5));
        Mat dilated = new Mat();
        Imgproc.dilate(combinedEdges, dilated, dilationElement);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(dilated, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        RotatedRect[] rects = new RotatedRect[contours.size()];
        double[] normalizedAngles = new double[contours.size()];

        for(int i = 0; i < contours.size(); i++) {
            MatOfPoint cnt = contours.get(i);

            double area = Imgproc.contourArea(cnt);
            if(area < 3000.0 || area > 7000.0) {
                continue;
            }

            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(cnt.toArray()));
            rects[i] = rect;
            normalizedAngles[i] = rect.size.height > rect.size.width ? 90.0 - rect.angle : -rect.angle;

            if(rect.size.width == 0 || rect.size.height == 0) continue;
            double ratio = Math.max(rect.size.width, rect.size.height) / Math.min(rect.size.width, rect.size.height);
            if(ratio < 2.0 || ratio > 3.0) continue;

//            List<Mat> channels = new ArrayList<>();
//            Core.split(hsv, channels);
//
        }

        frame.release();
        undistorted.release();
        wbCorrected.release();
        transformed.release();
        hsv.release();
        saturationThresh.release();
        satMasked.release();
        satEdges.release();
        edges.release();
        combinedEdges.release();
        dilated.release();
        hierarchy.release();

        processingTimeMillis = (System.nanoTime() - startNanoTime) / 1e6;
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Mat undistort(Mat frame) {
        Mat newCameraMatrix = Calib3d.getOptimalNewCameraMatrix(K, DIST_COEFFS, frame.size(), 1, frame.size());

        Mat undistorted = new Mat();
        Calib3d.undistort(frame, undistorted, K, DIST_COEFFS, newCameraMatrix);
        return undistorted;
    }

    public Mat applyGrayWorldWhiteBalance(Mat src) {
        // Ensure the image is in 3-channel BGR format
        if (src.channels() != 3) {
            throw new IllegalArgumentException("Input image must have 3 channels (BGR)");
        }

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

    private Vector2d getPosition(RotatedRect rect) {
        double pixelOffsetX = rect.center.x - REF_TOP_LEFT_PIXELS[0];
        double pixelOffsetY = rect.center.y - REF_TOP_LEFT_PIXELS[1];

        return new Vector2d(
                REF_TOP_LEFT_INCHES[0] + pixelOffsetX / PIXELS_PER_INCH,
                REF_TOP_LEFT_INCHES[1] + pixelOffsetY / PIXELS_PER_INCH
        );
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
}
