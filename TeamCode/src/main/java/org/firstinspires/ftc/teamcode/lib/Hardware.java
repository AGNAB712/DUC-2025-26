package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    private HardwareMap hwMap;
    public Intake intake1;
    public Intake intake2;
    public static Sorter sorter;
    public enum ArtifactType {
        GREEN,
        PURPLE,
        NONE
    }

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        sorter.init(new SimpleServo(hwMap, "sorter", 0, 360));
        intake1.init(new CRServo(hwMap, "intake1"), new SensorColor(hwMap, "color1"));
        intake2.init(new CRServo(hwMap, "intake2"), new SensorColor(hwMap, "color1"));
    }

    public static class Intake {
        public CRServo intakeContServo;
        public IntakeSensor colorSensor;
        public boolean isRotating;
        public void init(CRServo myServo, SensorColor myColorSensor) {
            this.intakeContServo = myServo;
            this.isRotating = false;
            this.colorSensor.init(myColorSensor);
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

        public void init(ServoEx myServo) {
            this.sorterServo = myServo;
        }
        public void green() {this.sorterServo.turnToAngle(greenPosition);}
        public void purple() {this.sorterServo.turnToAngle(purplePosition);}
        public void neutral() {this.sorterServo.turnToAngle(neutralPosition);}
    }
    public static class IntakeSensor {
        public static SensorColor sensor;
        public static int[] greenThresholdsHSV = {1, 1, 1};
        public static int[] purpleThresholdsHSV = {1, 1, 1};
        public static int distanceAllowed = 10;
        public static void init(SensorColor mySensor) {
            sensor = mySensor;
        }
        public static float[] getHSV() {
            float[] hsvValues = {};
            sensor.RGBtoHSV(sensor.red(), sensor.green(), sensor.blue(), hsvValues);
            return hsvValues;
        }
        public static ArtifactType detectColor() {
            float[] hsvValues = getHSV();
            int greenMatch = 0;
            int purpleMatch = 0;

            for (int i = 0; i < greenThresholdsHSV.length; i++) {
                if (greenThresholdsHSV[i] - distanceAllowed < hsvValues[i] && hsvValues[i] < greenThresholdsHSV[i] + distanceAllowed) {
                    greenMatch++;
                }
            }
            for (int i = 0; i < purpleThresholdsHSV.length; i++) {
                if (purpleThresholdsHSV[i] - distanceAllowed < hsvValues[i] && hsvValues[i] < purpleThresholdsHSV[i] + distanceAllowed) {
                    purpleMatch++;
                }
            }

            if (greenMatch == 3) {
                return ArtifactType.GREEN;
            } else if (purpleMatch == 3) {
                return ArtifactType.PURPLE;
            } else {
                return ArtifactType.NONE;
            }
        }
    }
}
