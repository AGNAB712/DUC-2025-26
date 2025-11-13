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

public class ducProcessorArtifacts implements VisionProcessor {

    public Scalar redLower = new Scalar(82.2, 66.8, 131.8);
    public Scalar redUpper = new Scalar(134.6, 255.0, 255.0);
    public Scalar greenLower = new Scalar(59.0, 53.0, 49.0);
    public Scalar greenUpper = new Scalar(93.5, 255.0, 222.4);
    public Scalar purpleLower = new Scalar(136.0, 48.2, 70.8);
    public Scalar purpleUpper = new Scalar(160.0, 255.0, 255.0);

    public Rect theFirstOne = new Rect(0, 0, 40, 40);
    public Rect theSecondOne = new Rect(0, 0, 40, 40);
    public Rect theThirdOne = new Rect(0, 0, 40, 40);
    public Rect mainRect;

    public Mat redFirst = new Mat();
    public Mat redSecond = new Mat();
    public Mat redThird = new Mat();
    public Mat greenMat = new Mat();
    public Mat purpleMat = new Mat();


    boolean tuning = false;
    public Mat hsvPurple = new Mat();
    public Mat hsvGreen = new Mat();
    public Mat thresholdGreen = new Mat();
    public Mat thresholdPurple = new Mat();

    public double camWidth = 0;
    public double camHeight = 0;

    public ArrayList<MatOfPoint> contoursGreen = new ArrayList<>();
    public ArrayList<MatOfPoint> contoursPurple = new ArrayList<>();

    public double contourAreaGreen = 0;
    public double contourAreaPurple = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        camHeight = height;
        camWidth = width;
        int mainRectHeight = 80;

        mainRect = new Rect(0, 0, width, height);
    }

    @Override
    public Object processFrame(Mat frameMain, long captureTimeNanos) {

        Mat frameGreen = frameMain.clone();
        Mat framePurple = frameMain.clone();

        if (tuning) {
            Imgproc.cvtColor(frameMain, frameMain, Imgproc.COLOR_RGB2HSV);
            Core.inRange(frameMain, purpleLower, purpleUpper, frameMain);
            greenMat = new Mat(frameMain, mainRect);

            //Imgproc.cvtColor(framePurple, framePurple, Imgproc.COLOR_RGB2HSV);
            //Core.inRange(frameMain, purpleLower, purpleUpper, frameMain);
            purpleMat = new Mat(frameMain, mainRect);

            Imgproc.rectangle(frameMain, mainRect, new Scalar(100,0,222));
            //Imgproc.rectangle(framePurple, mainRect, new Scalar(100,0,222));

            contourAreaGreen = 0;
            //contourAreaPurple = 0;

            //detectContours(purpleMat, mainRect, contoursPurple, framePurple, frameMain, false);
        } else {
            Imgproc.cvtColor(frameGreen, hsvGreen, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvGreen, greenLower, greenUpper, thresholdGreen);
            greenMat = new Mat(thresholdGreen, mainRect);

            Imgproc.cvtColor(framePurple, hsvPurple, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvPurple, purpleLower, purpleUpper, thresholdPurple);
            purpleMat = new Mat(thresholdPurple, mainRect);

            Imgproc.rectangle(frameMain, mainRect, new Scalar(100,0,222));

            contourAreaGreen = 0;
            contourAreaPurple = 0;

            //AREA 1
            //detectContours(redFirst, mainRect, contours, frame, 1);

            //AREA 2
            detectContours(greenMat, mainRect, contoursGreen, frameGreen, frameMain, true);
            detectContours(purpleMat, mainRect, contoursPurple, framePurple, frameMain, false);
        }



        return frameMain;
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

    private void detectContours(Mat box, Rect rectangle, ArrayList<MatOfPoint> contours, Mat frame, Mat masterFrame, boolean isGreen) {

        Imgproc.findContours(box, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (isGreen) {
            contourAreaGreen = 0;
        } else {
            contourAreaPurple = 0;
        }

        for (MatOfPoint contour : contours) {
            double width = calculateWidth(contour);
            double height = calculateHeight(contour);
            double area = width * height;

            if (isGreen) {
                if (area > contourAreaGreen && area < 150000 && area > 8000) {
                    contourAreaGreen = width * height;
                }
            } else {
                if (area > contourAreaPurple && area < 150000 && area > 8000) {
                    contourAreaPurple = width * height;
                }
            }


            width -= 2;
            height -= 2;

            Moments moments = Imgproc.moments(contour);
            double cX = moments.get_m10() / moments.get_m00();
            double cY = moments.get_m01() / moments.get_m00();

            Scalar color;
            if (isGreen) {
                color = new Scalar(0,240,0);
            } else {
                color = new Scalar(120,0,120);
            }

            Imgproc.rectangle(
                    masterFrame,
                    new Point(cX - (width/2) + rectangle.x, cY - (height/2) + rectangle.y),
                    new Point(cX + (width/2) + rectangle.x, cY + (height/2) + rectangle.y),
                    color,
                    2
            );
        }

        Imgproc.putText(masterFrame, Double.toString(contourAreaGreen), new Point(rectangle.x, 400), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255,255,255));

        contours.clear();

    }

    public double getContourGreen() {
        return contourAreaGreen;
    }
    public double getContourPurple() {
        return contourAreaPurple;
    }
}