package org.firstinspires.ftc.teamcode.lib;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.ducProcessorArtifacts;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;

@Configurable
public class Hardware {

    private HardwareMap hwMap;
    //public Intake intake1 = new Intake();
    //public Intake intake2 = new Intake();
    public Sorter sorter;
    public Shooter shooter;
    public Intake intakeFront;
    public static List<ArtifactType> sequence = new ArrayList<>();
    public enum ArtifactType {
        GREEN,
        PURPLE,
        NONE
    }
    public enum Teams {
        RED,
        BLUE
    }

    public BlackboardObject teamBlackboard = new BlackboardObject("Team");
    public BlackboardObject sequenceBlackboard = new BlackboardObject("Artifact");

    public List<ArtifactType> getCurrentArtifacts() {
        return sequence;
    }
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        sorter = new Sorter(new SimpleServo(hwMap, "sorter", 0, 180));

        intakeFront = new Intake(new CRServo(hwMap, "intakeFront"), hwMap.get(WebcamName.class, "Webcam 1"));

        shooter = new Shooter( //yaw, yaw encoder, pitch, launcher motor
                new CRServo(hwMap, "yaw1"),
                hwMap.get(AnalogInput.class, "yaw1_encoder"),
                new SimpleServo(hwMap, "pitch1", 0, 360),
                new MotorEx(hwMap, "launcherOne")
        );
    }

    //INTAKE
    //intake class includes:
    //the actual intake servo
    //the camera for color + contour detection
    public static class Intake {
        public CRServo intakeContServo;
        public IntakeSensor colorSensor;
        public boolean isRotating;
        public Intake(CRServo myServo, WebcamName webcam) {
            this.intakeContServo = myServo;
            this.isRotating = false;
            intakeContServo.setInverted(true);
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

    //SHOOTER
    //shooter class includes:
    //yaw servo, pitch servo
    //math for pointing to a position
    public static class Shooter {
        public double yaw = 0;
        public double pitch = 0;
        public crAxonServo yawServo;
        public ServoEx pitchServo;
        public MotorEx launcherMotor;
        private double percentPerFullYawServoRotation = 18/80;

        public Shooter(CRServo myYawServo, AnalogInput myYawServoEncoder, ServoEx myPitchServo, MotorEx myLauncherMotor) {
            yawServo = new crAxonServo(myYawServo, myYawServoEncoder);
            pitchServo = myPitchServo;
            launcherMotor = myLauncherMotor;
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

        double yawDegreesToYawTicks(double yawDegrees) {
            double ticksPerFullGearRotation = (360 / percentPerFullYawServoRotation);
            //ppfysr is the gear ratio (ie 18 tooth gear into 80 tooth gear)
            //this ends up being 1600 but i can change it easy if the gear ratios change

            return (yawDegrees/360.0 * ticksPerFullGearRotation);
        }

        public void setLauncherPower(double power) {
            launcherMotor.set(power);
        }


        void updateYaw(double yawToPoint) {
            yawServo.runToEncoderPosition(yawDegreesToYawTicks(yawToPoint));
            yaw = yawToPoint;
        }
        public void updatePitch(double pitchToPoint) {
            pitchServo.turnToAngle(pitchToPoint);
            pitch = pitchToPoint;
        }
    }

    //SORTER
    //sorter class includes:
    //the sorter servo
    public static class Sorter {
        public ServoEx sorterServo;
        public double greenPosition = 60;
        public double purplePosition = 110;
        public double neutralPosition = 90;
        private int noneCount = 0;
        private final int noneThreshold = 15;
        public ArtifactType currentColor = ArtifactType.NONE;

        public Sorter(ServoEx myServo) {
            this.sorterServo = myServo;
        }
        public void updateServo(ArtifactType detectedColor) {
            if (detectedColor != ArtifactType.NONE) {
                noneCount = 0;

                // only add when coming from "none" to a color
                if (detectedColor == ArtifactType.NONE) {
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

            if (currentColor == ArtifactType.GREEN) {
                this.green();
            } else if (currentColor == ArtifactType.PURPLE) {
                this.purple();
            } else {
                this.neutral();
            }
        }
        public void green() {this.sorterServo.turnToAngle(greenPosition);}
        public void purple() {this.sorterServo.turnToAngle(purplePosition);}
        public void neutral() {this.sorterServo.turnToAngle(neutralPosition);}
    }

    //INTAKE SENSOR
    //intake sensor class includes:
    //the webcam and processor for color + contour detection
    public static class IntakeSensor {
        private static ducProcessorArtifacts processor;
        public ArtifactType currentColor = ArtifactType.NONE;
        private int noneCount = 0;
        private final int noneThreshold = 15;

        public IntakeSensor(WebcamName webcam) {
            this.processor = new ducProcessorArtifacts();
            VisionPortal visionPortal = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .addProcessor(this.processor)
                    .build();
            ;
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
    }

    //CRAXONSERVO
    //qol class to convert a crservo to use its corresponding voltage meter as a tick measurement
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
            yawServoDirection = -1 * (TARGET_POS-totalAngle)/360;

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
            yawServoDirection = -1 * (TARGET_POS-totalAngle)/360;
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

    }

    public static class BlackboardObject {
        public static String KEY;
        public Object myObject;
        public BlackboardObject(String myKey) {
            KEY = myKey;
        }

        public Object get() {
            myObject = blackboard.getOrDefault(KEY, null);
            return myObject;
        }
        public void set(Object newThing) {
            blackboard.put(KEY, newThing);
            myObject = newThing;
        }
    }


}
