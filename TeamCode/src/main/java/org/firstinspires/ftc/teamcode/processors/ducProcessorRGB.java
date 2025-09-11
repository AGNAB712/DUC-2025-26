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
    private static final Scalar PURPLE = new Scalar(128, 0, 127);


    Point mainScanAreaTopLeft = new Point(50, 140);
    Point mainScanAreaBottomRight = new Point(150, 240);


    Mat scanArea_Cb;

    private volatile int averageBlue;
    private volatile int averageGreen;
    private volatile int averageRed;

    private volatile TYPE type = TYPE.TEAM_ELEMENT;
    private  volatile TYPE2 type2 = TYPE2.NOT_FOUND;
    private  volatile TYPE3 type3 = TYPE3.level3;


    //  private void inputToCb(Mat input) {
    //    Core.extractChannel(input, Cb, 1);
    //}

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        //inputToCb(input);


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

        Imgproc.putText(input, Double.toString(averageGreen), new Point(0, 400), Imgproc.FONT_HERSHEY_COMPLEX, 1, GREEN);
        Imgproc.putText(input, Double.toString(averageRed), new Point(100, 400), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(255, 0, 0));
        Imgproc.putText(input, Double.toString(averageBlue), new Point(200, 400), Imgproc.FONT_HERSHEY_COMPLEX, 1, BLUE);

        return input;
    }


    public TYPE getType() {
        return type;
    }
    public TYPE2 getType2(){
        return  type2;
    }
    public TYPE3 getType3(){
        return  type3;
    }
    public enum TYPE {
        TEAM_ELEMENT, NOTHING
    }
    public enum  TYPE2{
        LEFT,MIDDLE,RIGHT,NOT_FOUND
    }
    public enum  TYPE3{
        level1, level2,level3
    }

}