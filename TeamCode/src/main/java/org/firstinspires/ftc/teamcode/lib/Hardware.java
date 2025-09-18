package org.firstinspires.ftc.teamcode.lib;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.ducProcessorArtifacts;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Configurable
public class Hardware {

    private HardwareMap hwMap;
    //public Intake intake1 = new Intake();
    //public Intake intake2 = new Intake();
    public static IntakeSensor csensor1;
    public Sorter sorter;
    public Shooter shooter;
    public static List<ArtifactType> sequence = new ArrayList<>();
    public static double myP = -1;
    public enum ArtifactType {
        GREEN,
        PURPLE,
        NONE
    }
    public enum Teams {
        RED,
        BLUE
    }
    public static final String TEAM_KEY = "Team";
    Object team = blackboard.getOrDefault(TEAM_KEY, Teams.RED);

    public List<ArtifactType> getCurrentArtifacts() {
        return sequence;
    }
    public Object getCurrentTeam() {
        return team;
    }
    public void setTeam(Teams newTeam) {
        blackboard.put(TEAM_KEY, newTeam);
        team = newTeam;
    }

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        sorter = new Sorter(new SimpleServo(hwMap, "sorter", 0, 180));
        csensor1 = new IntakeSensor(hwMap.get(WebcamName.class, "Webcam 1"));

        shooter = new Shooter(new CRServo(hwMap, "yaw1"), hwMap.get(AnalogInput.class, "yaw1_encoder"));



        //intake1.init(new CRServo(hwMap, "intake1"), new SensorColor(hwMap, "color1"));
        //intake2.init(new CRServo(hwMap, "intake2"), new SensorColor(hwMap, "color2"));
    }

    public static class Intake {
        public CRServo intakeContServo;
        public IntakeSensor colorSensor;
        public boolean isRotating;
        public void init(CRServo myServo, WebcamName webcam) {
            this.intakeContServo = myServo;
            this.isRotating = false;
            this.stop();
            this.colorSensor = new IntakeSensor(webcam);
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
        private static VisionPortal visionPortal;
        private static ducProcessorArtifacts processor;
        public ArtifactType currentColor = ArtifactType.NONE;
        private int noneCount = 0;
        private final int noneThreshold = 15;

        public IntakeSensor(WebcamName webcam) {
            this.processor = new ducProcessorArtifacts();
            this.visionPortal = visionPortal = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .addProcessor(this.processor)
                    .build();;
        }

        public double[] contourAmount() {
            return (new double[] {this.processor.getContourGreen(), this.processor.getContourPurple()});
        }
        public ArtifactType detectColor() {
            double greenContourAmount = this.processor.getContourGreen();
            double purpleContourAmount = this.processor.getContourPurple();
            ArtifactType toReturn = ArtifactType.NONE;
            if (greenContourAmount == 0 && purpleContourAmount == 0) {
                toReturn = ArtifactType.NONE;
            } else if (greenContourAmount > purpleContourAmount) {
                toReturn = ArtifactType.GREEN;
            } else if (purpleContourAmount > greenContourAmount) {
                toReturn = ArtifactType.PURPLE;
            }

            return toReturn;
        }

        public void trackColor(ArtifactType detectedColor) {
            if (detectedColor != ArtifactType.NONE) {
                noneCount = 0;

                // only add when coming from "none" to a color
                if (currentColor == ArtifactType.NONE) {
                    currentColor = detectedColor;
                    sequence.add(currentColor);
                } else {
                    currentColor = detectedColor;
                }
            } else {
                noneCount++;
                if (noneCount >= noneThreshold) {
                    currentColor = ArtifactType.NONE;
                }
            }
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

    public static class crAxonServo {
        CRServo axonServo;
        AnalogInput axonServoEncoder;
        private double yawServoLastRecorded;
        double yawServoRotationServo;
        double yawServoDirection = 1;
        double lastDirection = 0;
        private double delta = 0;
        private boolean hasRotBeenAdded = false;
        public double rots = 0;
        public double totalAngle;
        public double TARGET_POS = 180;
        List<Double> distanceList = new ArrayList<>();
        public crAxonServo(CRServo myYawServo, AnalogInput myYawServoEncoder) {
            axonServo = myYawServo;
            axonServoEncoder = myYawServoEncoder;
        }

        public void update() {
            yawServoLastRecorded = yawServoRotationServo;
            yawServoRotationServo = (axonServoEncoder.getVoltage() / 3.3);
            delta = yawServoRotationServo - yawServoLastRecorded;
            if (yawServoDirection < 0) {
                delta=delta*-1;
            }

            lastDirection = yawServoDirection;
            yawServoDirection = myP * (TARGET_POS-totalAngle)/360;

            if (
                    delta > 0 && Math.abs(delta) >0.01
                    && !hasRotBeenAdded
                    && !(yawServoDirection > 0 && lastDirection < 0)
                    && !(yawServoDirection < 0 && lastDirection > 0)
            ){
                if (yawServoDirection > 0) {
                    rots--;
                } else {
                    rots++;
                }
                hasRotBeenAdded = true;
            } else {
                hasRotBeenAdded = false;
            }

            totalAngle = getTotalYawEncoder();

            lastDirection = yawServoDirection;
            yawServoDirection = myP * (TARGET_POS-totalAngle)/360;
            startRotation();

        }

        public double getTotalYawEncoder() {
            return (rots * 360) + (yawServoRotationServo * 360);
        }

        public void runToEncoderPosition(double runTo) {
            TARGET_POS = runTo;
        }

        public void startRotation() {
            axonServo.set(yawServoDirection);
        }

        public double[] showTelemetryData() {
            return new double[]{totalAngle, rots, TARGET_POS-totalAngle, yawServoDirection};
        }
        public List<Double> showOtherTelemetryData() {
            return distanceList;
        }


    }

    public static class Shooter {
        public double yaw = 0;
        public double pitch = 0;
        public double rots = 0;
        public crAxonServo yawServo;

        public Shooter(CRServo myYawServo, AnalogInput myYawServoEncoder) {
            yawServo = new crAxonServo(myYawServo, myYawServoEncoder);
        }

        void pointToPosition(Pose positionToPoint, Pose currentPosition, double robotHeading, Vector robotVelocity) {
            Pose projectedPosition = new Pose(currentPosition.getX() + robotVelocity.getXComponent(), currentPosition.getY() + robotVelocity.getYComponent());
            double theta = Math.toDegrees(
                    Math.atan(
                            (positionToPoint.getX() - currentPosition.getX())
                            /
                            (positionToPoint.getY() - currentPosition.getY())
                    )
            ) - robotHeading;
            updateYaw(theta);
        }
        void updateYaw(double yawToPoint) {
            //add servo logic here later
            yaw = yawToPoint;
        }
    }
}
