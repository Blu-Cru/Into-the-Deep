package org.firstinspires.ftc.teamcode.blucru.common.subsystems.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.blucru.common.util.Alliance;
import org.firstinspires.ftc.teamcode.blucru.common.util.Globals;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class PropDetectionProcessor implements VisionProcessor {


    /*
    0    -----------------------------------------
         |                                       |
         |                                       |
         |                                       |
         |                                       |
 y       |                                       |
         |                                       |
         |                                       |
         |                                       |
     1   -----------------------------------------
         0                                       1
                             x
     rectangle positions */
    public static double rect0x = 0.04;
    public static double rect0y = 0.75;
    public static double rect1x = 0.37;
    public static double rect1y = 0.71;
    public static double rect2x = 0.75;
    public static double rect2y = 0.75;

    // hues for blue
    public static double blueLowH = 90;
    public static double blueHighH = 140;

    // hues for red
    // since red wraps around 0, we need to use 2 ranges
    public static double redLowH1 = 0;
    public static double redHighH1 = 30;
    public static double redLowH2 = 150;
    public static double redHighH2 = 180;

    // make rectangles
    private static Rect rect0 = getRect(rect0x, rect0y, 100, 100, 1280, 720); // rectangle over left prop
    private static Rect rect1 = getRect(rect1x, rect1y, 100, 100, 1280, 720); // rectangle over middle prop
    private static Rect rect2 = getRect(rect2x, rect2y, 100, 100, 1280, 720); // rectangle over right prop

    private Mat subMat0, subMat1, subMat2;

    // average value in each rectangle
    public double average0, average1, average2;
    private double max;
    ArrayList<double[]> frameList;
    public static double strictLowS = 50; // low saturation value for strict HSV filter
    public static double strictHighS = 255; // high saturation value for strict HSV filter
    public int position = 1;

    public static Paint green = new Paint();

    public PropDetectionProcessor() {
        green.setColor(Color.rgb(0, 255, 0));
        green.setStyle(Paint.Style.STROKE);
        green.setStrokeWidth(5);
        green.setAntiAlias(true);
    }

    public void init(int width, int height, CameraCalibration cameraCalibration) {

    }

    public Mat processFrame(Mat inputRGB, long captureTime) {
        Mat inputHSV = new Mat();

        Imgproc.cvtColor(inputRGB, inputHSV, Imgproc.COLOR_RGB2HSV);
        if(inputHSV.empty()) {
            return inputRGB;
        }

        Mat thresh = new Mat();

        //apply HSV filter
        if(Globals.alliance == Alliance.BLUE) {
            Scalar lower = new Scalar(blueLowH,40,20);
            Scalar upper = new Scalar(blueHighH, 255, 255);

            Core.inRange(inputHSV, lower, upper, thresh);
        } else {
            Scalar lower1 = new Scalar(redLowH1, 40, 20);
            Scalar upper1 = new Scalar(redHighH1, 255, 255);

            Scalar lower2 = new Scalar(redLowH2, 40, 20);
            Scalar upper2 = new Scalar(redHighH2, 255, 255);

            Mat thresh1 = new Mat();
            Mat thresh2 = new Mat();

            Core.inRange(inputHSV, lower2, upper2, thresh2);
            Core.inRange(inputHSV, lower1, upper1, thresh1);

            // combine the 2 red threshes
            Core.bitwise_or(thresh1, thresh2, thresh);

            // release thresh1 and thresh2 to save memory
            thresh1.release();
            thresh2.release();
        }

        Mat masked = new Mat();
        Core.bitwise_and(inputHSV, inputHSV, masked, thresh);

        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);
        Mat scaledMask = new Mat();

        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);

        Mat scaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for blue
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for blue
        //apply strict HSV filter onto scaledMask to get rid of any red/blue other than the prop
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        // draw rectangles on scaledThresh
        Imgproc.rectangle(scaledThresh, rect0, new Scalar(255, 255, 255), 2);
        Imgproc.rectangle(scaledThresh, rect1, new Scalar(255, 255, 255), 2);
        Imgproc.rectangle(scaledThresh, rect2, new Scalar(255, 255, 255), 2);

        // create submats for 3 detection areas
        subMat0 = scaledThresh.submat(rect0);
        subMat1 = scaledThresh.submat(rect1);
        subMat2 = scaledThresh.submat(rect2);

        // get average values of each submat
        average0 = Core.mean(subMat0).val[0];
        average1 = Core.mean(subMat1).val[0];
        average2 = Core.mean(subMat2).val[0];

        // find the max average value
        max = Math.max(Math.max(average0, average1), average2);
        // set position based on max average value
        if(max == average0) {
            position = 0;
        } else if(max == average1) {
            position = 1;
        } else {
            position = 2;
        }

        // arraylist for mean filter (not used)
//        if (frameList.size() > 5) {
//            frameList.remove(0);
//        }

        Imgproc.cvtColor(masked, masked, Imgproc.COLOR_HSV2RGB);

        // input.release();
        scaledThresh.release();
//        scaledMask.release();
        inputHSV.release();
        masked.release();
        thresh.release();
        //change the return to whatever mat you want in camera stream
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
        return scaledMask;
    }

    public void onDrawFrame(Canvas canvas,
                            int width,
                            int height,
                            float bmpToCanvasPx,
                            float bmpDensity,
                            Object userContext) {
        canvas.drawRect(rect0.x * bmpToCanvasPx, rect0.y * bmpToCanvasPx, (rect0.x + rect0.width) * bmpToCanvasPx, (rect0.y + rect0.height) * bmpToCanvasPx, green);
        canvas.drawRect(rect1.x * bmpToCanvasPx, rect1.y * bmpToCanvasPx, (rect1.x + rect1.width) * bmpToCanvasPx, (rect1.y + rect1.height) * bmpToCanvasPx, green);
        canvas.drawRect(rect2.x * bmpToCanvasPx, rect2.y * bmpToCanvasPx, (rect2.x + rect2.width) * bmpToCanvasPx, (rect2.y + rect2.height) * bmpToCanvasPx, green);
    }

    public static Rect getRect(double centerx, double centery, int width, int height, int camWidth, int camHeight) {
        int subMatRectX = (int)(camWidth * centerx) - (width / 2);
        int subMatRectY = (int)(camHeight * centery) - (height / 2);
        int subMatRectWidth = width;
        int subMatRectHeight = height;

        Rect subMatRect = new Rect(subMatRectX, subMatRectY, subMatRectWidth, subMatRectHeight);
        return subMatRect;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("average 0", average0);
        telemetry.addData("average 1", average1);
        telemetry.addData("average 2", average2);
        telemetry.addData("POSITION", position);
    }
}
