package org.firstinspires.ftc.teamcode.lib;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import android.util.Size;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.ServoEx;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.processors.ducProcessorArtifacts;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Configurable
public class Hardware {

    private HardwareMap hwMap;
    public Follower follower;
    public MotorEx fL, fR, rL, rR;
    //public Intake intake1 = new Intake();
    //public Intake intake2 = new Intake();
    public Sorter sorter;
    public Shooter shooterRight;
    public Shooter shooterLeft;
    public Chute chuteRight;
    public Chute chuteLeft;
    public Intake intakeFront;
    public Intake intakeBack;
    public Lock lock;

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
    public enum servoPositions {
        ROTATING, STOPPED, REVERSED
    }

    public BlackboardObject teamBlackboard = new BlackboardObject("Team");
    public BlackboardObject sequenceBlackboard = new BlackboardObject("Artifact");
    public BlackboardObject endPositionBlackboard = new BlackboardObject("EndPosition");

    public List<ArtifactType> getCurrentArtifacts() {
        return sequence;
    }
    public Hardware(HardwareMap hwMap) {
        this.hwMap = hwMap;

        //expansion - servos
        //0 intakeFront
        //1 intakeBack
        //2 rightChute
        //3 leftChute
        //expansion - motors
        //0 frontLeft
        //1 backLeft
        //2 shooterRight

        //control - servos
        //0 sorter
        //1 lock
        //2 pitchLeft
        //3 pitchRight
        //control - motors
        //0 frontRight
        //1 backRight
        //2 shooterLeft

        intakeFront = new Intake(new CRServo(hwMap, "intakeFront"), hwMap.get(WebcamName.class, "Webcam 1"), false);
        intakeBack = new Intake(new CRServo(hwMap, "intakeBack"), hwMap.get(WebcamName.class, "Webcam 2"), true);

        shooterRight = new Shooter(new SimpleServo(hwMap, "pitchRight", 0, 360), new MotorEx(hwMap, "shooterRight"), true);
        shooterLeft = new Shooter(new SimpleServo(hwMap, "pitchLeft", 0, 360), new MotorEx(hwMap, "shooterLeft"), false);

        chuteRight = new Chute(new CRServo(hwMap, "rightChute"));
        chuteLeft = new Chute(new CRServo(hwMap, "leftChute"));

        lock = new Lock(new SimpleServo(hwMap, "lock", 0, 180));
        sorter = new Sorter(new SimpleServo(hwMap, "sorter", 0, 180));

        /*VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 3"))
                //.addProcessor(new ducProcessorArtifacts())
                //.setLiveViewContainerId(LiveViewContainerId)
                .enableLiveView(false)
                .build();*/
    }

    public static class Chute extends SubsystemBase {
        public CRServo spinny;
        public boolean isRotating = false;

        public Chute(CRServo mySpinny) {
            spinny = mySpinny;
            spinny.setInverted(true);
        }

        public void setRotation(servoPositions newPosition) {
            if (newPosition == servoPositions.ROTATING) {
                this.isRotating = true;
                this.spinny.set(1);
            } else if (newPosition == servoPositions.REVERSED) {
                this.isRotating = true;
                this.spinny.set(-1);
            } else if (newPosition == servoPositions.STOPPED) {
                this.isRotating = false;
                this.spinny.stop();
            }
        }
        public void stop() {setRotation(servoPositions.STOPPED);}
        public void start() {setRotation(servoPositions.ROTATING);}
        public void reverse() {setRotation(servoPositions.REVERSED);}
    }

    public static class Lock extends SubsystemBase {
        public SimpleServo chuteLock;
        public Lock(SimpleServo myChuteLock) {
            chuteLock = myChuteLock;
        }
        public void open() {
            chuteLock.setPosition(0.25);
        }
        public void close() {
            chuteLock.setPosition(0);
        }
    }

    //INTAKE
    //intake class includes:
    //the actual intake servo
    //the camera for color + contour detection
    public class Intake extends SubsystemBase {
        public CRServo intakeContServo;
        public IntakeSensor colorSensor;
        public boolean isRotating;
        public Intake(CRServo myServo, WebcamName webcam, boolean LiveViewContainerId) {
            this.intakeContServo = myServo;
            this.isRotating = false;
            intakeContServo.setInverted(true);
            this.stop();
            this.colorSensor = new IntakeSensor(webcam, LiveViewContainerId);
        }

        public void setRotation(servoPositions newPosition) {
            if (newPosition == servoPositions.ROTATING) {
                this.isRotating = true;
                this.intakeContServo.set(1);
            } else if (newPosition == servoPositions.REVERSED) {
                this.isRotating = true;
                this.intakeContServo.set(-1);
            } else if (newPosition == servoPositions.STOPPED) {
                this.isRotating = false;
                this.intakeContServo.stop();
            }
        }
        public void stop() {setRotation(servoPositions.STOPPED);}
        public void start() {setRotation(servoPositions.ROTATING);}
        public void reverse() {setRotation(servoPositions.REVERSED);}
    }

    //SHOOTER
    //shooter class includes:
    //yaw servo, pitch servo
    //math for pointing to a position
    public static class Shooter extends SubsystemBase {
        public double pitch = 0;
        public ServoEx pitchServo;
        public MotorEx launcherMotor;
        public double targetVelocity = 0;
        PIDFController velocityPIDController = new PIDFController(new PIDFCoefficients(1, 0, 0.3, 0));

        public Shooter(ServoEx myPitchServo, MotorEx myLauncherMotor, boolean isInverted) {
            pitchServo = myPitchServo;
            launcherMotor = myLauncherMotor;
            launcherMotor.setInverted(isInverted);
            launcherMotor.stopMotor();
        }


        public void setLauncherVelocity(double vel) {
            targetVelocity = vel;
        }
        public void keepLauncherAtVelocity() {
            double velError = targetVelocity - launcherMotor.getCorrectedVelocity();
            velocityPIDController.updateError(velError);
            launcherMotor.set(velocityPIDController.run());
        }
        public boolean isLauncherAtVelocity() {
            double currentVel = launcherMotor.getCorrectedVelocity();
            return (targetVelocity + 50) > currentVel && currentVel > (targetVelocity - 50);
        }
        public void stop() {
            launcherMotor.stopMotor();
        }

        public double getPitchAngle() {
            return ((pitchServo.getAngle() - 195) / 195) * -17;
        }
        public void setPitchAngle(double pitchToPoint) {
            double clampedAngle = clamp(pitchToPoint, 0, 17);
            pitchServo.turnToAngle((clampedAngle / 17) * 195);
        }
    }

    //SORTER
    //sorter class includes:
    //the sorter servo
    public static class Sorter extends SubsystemBase {
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
        public void updateServo(ArtifactType detectedColor, boolean isInverted) {
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
                if (isInverted) {this.purple();} else {this.green();}
            } else if (currentColor == ArtifactType.PURPLE) {
                if (isInverted) {this.green();} else {this.purple();}
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
    public class IntakeSensor extends SubsystemBase {
        public ducProcessorArtifacts processor;
        public ArtifactType currentColor = ArtifactType.NONE;
        private int noneCount = 0;
        private final int noneThreshold = 15;
        VisionPortal visionPortal;

        public IntakeSensor(WebcamName webcam, boolean liveView) {
            this.processor = new ducProcessorArtifacts();
            //.setLiveViewContainerId(LiveViewContainerId)
            visionPortal = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .addProcessor(this.processor)
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    //.setLiveViewContainerId(LiveViewContainerId)
                    .enableLiveView(false)
                    .setCameraResolution(new Size(640, 480))
                    .build();
            ;

        }
        public VisionPortal.CameraState getCamera() {
            return visionPortal.getCameraState();
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

    public static double clamp(double value, double min, double max) {
        if (value > max) {
            return max;
        } else if (value < min) {
            return min;
        } else {
            return value;
        }
    }


}
