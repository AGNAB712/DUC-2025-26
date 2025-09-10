package org.firstinspires.ftc.teamcode.lib;

import android.graphics.Color;
import android.util.Size;

import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.ducProcessorArtifactsGreen;
import org.firstinspires.ftc.teamcode.processors.ducProcessorArtifactsPurple;
import org.firstinspires.ftc.vision.VisionPortal;

public class Hardware {

    private HardwareMap hwMap;
    //public Intake intake1 = new Intake();
    //public Intake intake2 = new Intake();
    public static IntakeSensor csensor1;
    public static Sorter sorter;
    private static ducProcessorArtifactsGreen ducProcessorGreen;
    private ducProcessorArtifactsPurple ducProcessorPurple;
    public VisionPortal visionPortal;
    public enum ArtifactType {
        GREEN,
        PURPLE,
        NONE
    }

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        sorter = new Sorter(new SimpleServo(hwMap, "sorter", 0, 360));
        csensor1 = new IntakeSensor(new SensorColor(hwMap, "color1"));

        ducProcessorGreen = new ducProcessorArtifactsGreen();
        ducProcessorPurple = new ducProcessorArtifactsPurple();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(ducProcessorGreen)
                .addProcessor(ducProcessorPurple)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}
        //intake1.init(new CRServo(hwMap, "intake1"), new SensorColor(hwMap, "color1"));
        //intake2.init(new CRServo(hwMap, "intake2"), new SensorColor(hwMap, "color2"));
    }

    public static class Intake {
        public CRServo intakeContServo;
        public IntakeSensor colorSensor;
        public boolean isRotating;
        public void init(CRServo myServo, SensorColor myColorSensor) {
            this.intakeContServo = myServo;
            this.isRotating = false;
            this.stop();
            this.colorSensor = new IntakeSensor(myColorSensor);
        }

        public void setRotation(boolean rotationSet) {
            this.isRotating = rotationSet;
            if (this.isRotating) {
                this.intakeContServo.set(1);
            } else {
                this.intakeContServo.stop();
            }
        }
        public void stop() {setRotation(false);}
        public void start() {setRotation(true);}
    }
    public static class Sorter {
        public ServoEx sorterServo;
        public double greenPosition = 45;
        public double purplePosition = -45;
        public double neutralPosition = 0;

        public Sorter(ServoEx myServo) {
            this.sorterServo = myServo;
        }
        public void updateServo(ArtifactType type) {
            if (type == ArtifactType.GREEN) {
                this.green();
            } else if (type == ArtifactType.PURPLE) {
                this.purple();
            } else {
                this.neutral();
            }
        }
        public void green() {this.sorterServo.turnToAngle(greenPosition);}
        public void purple() {this.sorterServo.turnToAngle(purplePosition);}
        public void neutral() {this.sorterServo.turnToAngle(neutralPosition);}
    }
    public static class IntakeSensor {
        private static SensorColor sensor;
        public static int[] greenThresholdsHSV = {1, 1, 1};
        public static int[] purpleThresholdsHSV = {1, 1, 1};
        public static int distanceAllowed = 100;
        public IntakeSensor(SensorColor mySensor) {
            this.sensor = mySensor;
        }

        public static float[] getHSV() {
            float[] hsvValues = {0F, 0F, 0F};
            Color.RGBToHSV(sensor.red() * 8, sensor.green()*8, sensor.blue()*8, hsvValues);
            return hsvValues;
        }
        public static float[] getRGB() {
            return new float[] {sensor.red(), sensor.green(), sensor.blue()};
        }
        public static ArtifactType detectColor() {
            double greenContourAmount = ducProcessorGreen.getContours();
            double purpleContourAmount = ducProcessorGreen.getContours();
            if (greenContourAmount == 0 && purpleContourAmount == 0) {
                return ArtifactType.NONE;
            } else if (greenContourAmount > purpleContourAmount) {
                return ArtifactType.GREEN;
            } else if (purpleContourAmount >= greenContourAmount) {
                return ArtifactType.PURPLE;
            }
            return ArtifactType.NONE;
        }
            /*float[] hsvValues = getHSV();
            int greenMatch = 0;
            int purpleMatch = 0;
            double[] greenThresholdHSVBottom = {80, 0.4, 0.014};
            double[] greenThresholdHSVTop = {160, 0.75, 0.024};

            double[] purpleThresholdHSVBottom = {30, 0.5, 0.019};
            double[] purpleThresholdHSVTop = {60, 0.75, 0.025};

            for (int i = 0; i < greenThresholdsHSV.length; i++) {
                if (greenThresholdHSVBottom[i] <= hsvValues[i] && hsvValues[i] <= greenThresholdHSVTop[i]) {
                    greenMatch++;
                }
            }
            for (int i = 0; i < purpleThresholdsHSV.length; i++) {
                if (purpleThresholdHSVBottom[i] <= hsvValues[i] && hsvValues[i] <= purpleThresholdHSVTop[i]) {
                    purpleMatch++;
                }
            }

            if (greenMatch >= 2) {
                return ArtifactType.GREEN;
            } else if (purpleMatch >= 2) {
                return ArtifactType.PURPLE;
            } else {
                return ArtifactType.NONE;
            }
        }*/
    }
}
