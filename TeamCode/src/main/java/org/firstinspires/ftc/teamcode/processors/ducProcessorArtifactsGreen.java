package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;

public class ducProcessorArtifactsGreen implements VisionProcessor {

    public Scalar redLower = new Scalar(82.2, 66.8, 131.8);
    public Scalar redUpper = new Scalar(134.6, 255.0, 255.0);
    public Scalar greenLower = new Scalar(93.0, 177.0, 55.0);
    public Scalar greenUpper = new Scalar(134.6, 255.0, 255.0);
    public Scalar purpleLower = new Scalar(121.0, 46.0, 0.0);
    public Scalar purpleUpper = new Scalar(255.0, 255.0, 255.0);

    public Rect theFirstOne = new Rect(0, 0, 40, 40);
    public Rect theSecondOne = new Rect(0, 0, 40, 40);
    public Rect theThirdOne = new Rect(0, 0, 40, 40);
    public Rect mainRect = new Rect(0, 0, 640, 480);

    public Mat redFirst = new Mat();
    public Mat redSecond = new Mat();
    public Mat redThird = new Mat();
    public Mat greenMat = new Mat();
    public Mat purpleMat = new Mat();


    boolean tuning = false;
    public Mat hsv = new Mat();
    public Mat threshold = new Mat();

    public ArrayList<MatOfPoint> contours = new ArrayList<>();

    public double contourArea = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (tuning) {
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
            Core.inRange(frame, greenLower, greenUpper, frame);
            greenMat = new Mat(frame, mainRect);
            purpleMat = new Mat(frame, mainRect);
        } else {
            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, greenLower, greenUpper, threshold);
            greenMat = new Mat(threshold, mainRect);
            purpleMat = new Mat(threshold, mainRect);
        }

        Imgproc.rectangle(frame, mainRect, new Scalar(100,0,222));

        contourArea = 0;

        //AREA 1
        //detectContours(redFirst, mainRect, contours, frame, 1);

        //AREA 2
        detectContours(purpleMat, mainRect, contours, frame, 2);

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    private double calculateHeight(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.height;
    }

    private void detectContours(Mat box, Rect rectangle, ArrayList<MatOfPoint> contours, Mat frame, double number) {

        Imgproc.findContours(box, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        contourArea = 0;

        for (MatOfPoint contour : contours) {
            double width = calculateWidth(contour);
            double height = calculateHeight(contour);
            double area = width * height;

            if (area > contourArea && area < 150000) {
                contourArea = width * height;
            }

            width -= 2;
            height -= 2;

            Moments moments = Imgproc.moments(contour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();

            Imgproc.rectangle(
                    frame,
                    new Point(cX - (width/2) + rectangle.x, cY - (height/2) + rectangle.y),
                    new Point(cX + (width/2) + rectangle.x, cY + (height/2) + rectangle.y),
                    new Scalar(240,240,240),
                    2
            );
        }

        Imgproc.putText(frame, Double.toString(contourArea), new Point(rectangle.x, 400), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255,255,255));

        contours.clear();

    }

    public double getContours() {
        return contourArea;
    }
}