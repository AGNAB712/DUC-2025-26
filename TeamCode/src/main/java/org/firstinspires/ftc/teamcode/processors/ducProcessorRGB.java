package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ducProcessorRGB implements VisionProcessor {
    private static final Scalar BLUE = new Scalar(0, 0, 255);
    private static final Scalar GREEN = new Scalar(0, 255, 0);
    private static final Scalar RED = new Scalar(255, 0, 0);

    private static final double[] purpleReference = new double[] {145, 96, 146};
    private static final double[] greenReference = new double[] {60, 146, 125};

    int offset = 65;
    Point mainScanAreaTopLeft = new Point(65, 55 + offset);
    Point mainScanAreaBottomRight = new Point(110, 105 + offset);


    Mat scanArea_Cb;

    private volatile int averageBlue;
    private volatile int averageGreen;
    private volatile int averageRed;
    private double[] averageColors = {0, 0, 0};

    int cameraWidth = 0;
    int cameraHeight = 0;



    //  private void inputToCb(Mat input) {
    //    Core.extractChannel(input, Cb, 1);
    //}

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        //inputToCb(input);
        cameraWidth = width;
        cameraHeight = height;

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        //    inputToCb(input);

        scanArea_Cb = input.submat(new Rect(mainScanAreaTopLeft, mainScanAreaBottomRight));
        averageGreen = (int) Core.mean(scanArea_Cb).val[1];
        averageRed = (int) Core.mean(scanArea_Cb).val[0];
        averageBlue = (int) Core.mean(scanArea_Cb).val[2];
        Imgproc.rectangle(input, mainScanAreaTopLeft, mainScanAreaBottomRight, BLUE, 2);

        Imgproc.putText(input, Double.toString(averageRed), new Point(0, 10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, RED);
        Imgproc.putText(input, Double.toString(averageGreen), new Point(50, 10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, GREEN);
        Imgproc.putText(input, Double.toString(averageBlue), new Point(100, 10), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, BLUE);
        averageColors = new double[] {averageRed, averageGreen, averageBlue};

        double distanceGreen = getContourGreen();
        double distancePurple = getContourPurple();
        Imgproc.putText(input, Double.toString(distanceGreen), new Point(0, 20), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(greenReference));
        Imgproc.putText(input, Double.toString(distancePurple), new Point(50, 20), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(purpleReference));

        return input;
    }


    public double getContourGreen() {
        double toReturn = 0;
        for (int i =0; i < greenReference.length; i++) {
            double distance = Math.abs(averageColors[i] - greenReference[i]);
            toReturn = toReturn+distance;
        }
        return toReturn;
    }

    public double getContourPurple() {
        double toReturnPurple = 0;
        for (int i =0; i < purpleReference.length; i++) {
            double distance = Math.abs(averageColors[i] - purpleReference[i]);
            toReturnPurple = toReturnPurple+distance;
        }
        return toReturnPurple;
    }


}