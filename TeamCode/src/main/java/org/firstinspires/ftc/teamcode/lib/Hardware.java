package org.firstinspires.ftc.teamcode.lib;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;

import android.util.Size;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Vector2d;
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
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.processors.ducProcessorArtifacts;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Configurable
public class Hardware {

    private HardwareMap hwMap;
    public MotorEx liftLeft, liftRight;
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
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    public Position cameraPosition = new Position(DistanceUnit.INCH, 0, 8, 12.5, 0);
    public YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, -90, 0);

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
    public Point[] pointsInBigTriangle = new Point[]{
            new Point(0, 144), new Point(144, 144), new Point(72, 72)
    };
    public Point[] pointsInSmallTriangle = new Point[]{
            new Point(50, 0), new Point(72, 22), new Point(95, 0)
    };
    public Point[] pointsInDangerZoneRed = new Point[]{
            new Point(143, 77), new Point(132, 77), new Point(143, 65), new Point(132, 65)
    };
    public Point[] pointsInDangerZoneBlue = new Point[]{
            new Point(0, 77), new Point(12, 77), new Point(0, 65), new Point(12, 65)
    };


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

        intakeFront = new Intake(new CRServo(hwMap, "intakeFront"), hwMap.get(HuskyLens.class, "lensFront"), false);
        intakeBack = new Intake(new CRServo(hwMap, "intakeBack"), hwMap.get(HuskyLens.class, "lensBack"), true);

        shooterRight = new Shooter(new SimpleServo(hwMap, "pitchRight", 0, 360), new MotorEx(hwMap, "shooterRight"), true);
        shooterLeft = new Shooter(new SimpleServo(hwMap, "pitchLeft", 0, 360), new MotorEx(hwMap, "shooterLeft"), false);

        chuteRight = new Chute(new CRServo(hwMap, "rightChute"));
        chuteLeft = new Chute(new CRServo(hwMap, "leftChute"));

        lock = new Lock(new SimpleServo(hwMap, "lock", 0, 360));
        sorter = new Sorter(new SimpleServo(hwMap, "sorter", 0, 360));

        liftLeft = new MotorEx(hwMap, "liftLeft");
        liftRight = new MotorEx(hwMap, "liftRight");

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public static class VelocityLUT {
        InterpLUT lut;
        InterpLUT powerLut;
        public VelocityLUT() {
            lut = new InterpLUT();
            lut.add(-1000, 1199-100);
            lut.add(29, 1200-100);
            lut.add(35, 1200-100);
            lut.add(55, 1325-100);
            lut.add(80, 1500-100);
            lut.add(111, 1850-300);
            lut.add(123, 1870-300);
            lut.add(1000, 1951-300); //the random -stuff here

            powerLut = new InterpLUT();
            powerLut.add(160, 0.1);
            powerLut.add(370, 0.2);
            powerLut.add(660, 0.3);
            powerLut.add(930, 0.4);
            powerLut.add(1140, 0.5);
            powerLut.add(1410, 0.6);
            powerLut.add(1640, 0.7);
            powerLut.add(1900, 0.8);
            powerLut.add(2120, 0.9);


            //35 should be max angle, 1200 vel
            //55 should be max angle, 1325 vel
            //80 should be max angle, 1500 vel

            lut.createLUT();
            powerLut.createLUT();
        }

        public double[] get(double distance) {
            double angleToReturn = 1;
            if (distance > 34) {
                angleToReturn = 0;
            }
            return new double[]{lut.get(distance), angleToReturn};
        }

        public double velocityToPower(double distance) {
            return powerLut.get(distance);
        }
    }

    public class Chute extends SubsystemBase {
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
        public boolean isOpen = false;
        public Lock(SimpleServo myChuteLock) {
            chuteLock = myChuteLock;
        }
        public void open() {
            isOpen = true;

            chuteLock.setPosition(0);
        }
        public void close() {
            isOpen = false;

            chuteLock.setPosition(0.69);
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
        public Intake(CRServo myServo, HuskyLens cam, boolean LiveViewContainerId) {
            this.intakeContServo = myServo;
            this.isRotating = false;
            intakeContServo.setInverted(true);
            this.stop();
            this.colorSensor = new IntakeSensor(cam, LiveViewContainerId);
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
        public double velError;
        public double powerAmount = 0;
        PIDFController velocityPIDController = new PIDFController(new PIDFCoefficients(0.025, 0, 0, 0));

        public Shooter(ServoEx myPitchServo, MotorEx myLauncherMotor, boolean isInverted) {
            pitchServo = myPitchServo;
            launcherMotor = myLauncherMotor;
            launcherMotor.setInverted(isInverted);
            launcherMotor.stopMotor();
            //launcherMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }


        public void setLauncherVelocity(double vel) {
            targetVelocity = vel;
        }
        public void keepLauncherAtVelocity() {
            if (launcherMotor.getCorrectedVelocity() < targetVelocity) {
                launcherMotor.set(1);
            } else {
                launcherMotor.set(0.65);
            }

        }
        public boolean isLauncherAtVelocity() {
            double currentVel = launcherMotor.getCorrectedVelocity();
            return (targetVelocity + 20) > currentVel && currentVel > (targetVelocity - 20);
        }
        public void stop() {
            launcherMotor.stopMotor();
        }

        public double getPitchAngle() {
            return ((pitchServo.getAngle() - 195) / 195) * -20;
        }
        public void setPitchAngle(double pitchToPoint, boolean isLeftSide) {
            if (isLeftSide) {
                pitchServo.setPosition((pitchToPoint * 0.5) + 0.5);
            } else {
                pitchServo.setPosition(((1-pitchToPoint) * 0.85) + 0.15);
            }
        }
    }

    //SORTER
    //sorter class includes:
    //the sorter servo
    public static class Sorter extends SubsystemBase {
        public ServoEx sorterServo;
        public double[] greenPositions = new double[]{0, 345};
        public double[] purplePositions = new double[]{110, 245};
        public double[] neutralPositions = new double[]{55, 290};
        private int noneCount = 0;
        private final int noneThreshold = 15;
        public boolean isInGreen = false;
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
                if (isInverted) {this.purple(isInverted);} else {this.green(isInverted);}
            } else if (currentColor == ArtifactType.PURPLE) {
                if (isInverted) {this.green(isInverted);} else {this.purple(isInverted);}
            } else {
                this.neutral();
            }
        }
        public void green(boolean inverted) {
            isInGreen = true;
            this.sorterServo.turnToAngle(inverted ? purplePositions[0] : purplePositions[0]);
        }
        public void purple(boolean inverted) {
            isInGreen = false;
            this.sorterServo.turnToAngle(inverted ? greenPositions[0] : greenPositions[0]);
        }
        public void neutral() {
            isInGreen = false;
            this.sorterServo.turnToAngle(neutralPositions[0]);
        }
    }

    //INTAKE SENSOR
    //intake sensor class includes:
    //the webcam and processor for color + contour detection
    public class IntakeSensor extends SubsystemBase {
        public HuskyLens huskyLens;
        public ArtifactType currentColor = ArtifactType.NONE;
        private int noneCount = 0;
        private final int noneThreshold = 15;

        public IntakeSensor(HuskyLens cam, boolean liveView) {
            huskyLens = cam;
            Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS);
            rateLimit.expire();
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        }
        public ArtifactType detectColor() {
            int purpleSize = 0;
            int greenSize = 0;
            ArtifactType artifactDetected = ArtifactType.NONE;
            HuskyLens.Block[] blocks = huskyLens.blocks();
            for (int i = 0; i < blocks.length; i++) {
                int area = (blocks[i].height * blocks[i].width);
                if (area > 5000) {
                    if (blocks[i].id == 1) {
                        greenSize = greenSize + area;
                    } else {
                        purpleSize = purpleSize + area;
                    }
                }
            }
            if (greenSize > purpleSize) {
                artifactDetected = ArtifactType.GREEN;
            } else if (greenSize < purpleSize) {
                artifactDetected = ArtifactType.PURPLE;
            }

            return artifactDetected;
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

    public class BlackboardObject {
        public String KEY;
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

    public static double distanceToGoal(Teams team, Pose position) {
        Pose poseToUse = team == Teams.BLUE ? new Pose(14, 130) : new Pose(130, 130);
        return Math.sqrt(
                Math.pow((position.getX() - poseToUse.getX()), 2)
                        +
                Math.pow((position.getY() - poseToUse.getY()), 2)
        );
    }



    public Point[] rotateSquare(double centerX, double centerY, double heading) {

        Point[] originalVertices = new Point[] {
                new Point(centerX - 9, centerY - 9),
                new Point(centerX + 9, centerY - 9),
                new Point(centerX + 9, centerY + 9),
                new Point(centerX - 9, centerY + 9)
        };

        double cosineOfAngle = Math.cos(heading);
        double sineOfAngle = Math.sin(heading);

        Point[] rotatedVertices = new Point[4];

        for (int i = 0; i < 4; i++) {
            double translatedX = originalVertices[i].x - centerX;
            double translatedY = originalVertices[i].y - centerY;

            double rotatedX = translatedX * cosineOfAngle - translatedY * sineOfAngle;
            double rotatedY = translatedX * sineOfAngle + translatedY * cosineOfAngle;

            double finalX = rotatedX + centerX;
            double finalY = rotatedY + centerY;

            rotatedVertices[i] = new Point(finalX, finalY);
        }

        return rotatedVertices;
    }

    class Point {
        public double x;
        public double y;
        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    public boolean isInTriangle(Pose position) {
        //there are simpler ways to do this but i wanted to do this cause its cool
        double botX = position.getX();
        double botY = position.getY();
        double botHeading = position.getHeading();
        boolean isColliding;
        Point[] pointsOnBot = rotateSquare(botX, botY, botHeading);


        for (int i =0; i < pointsOnBot.length; i++) {
            Point point0 = pointsOnBot[i];
            Point point1 = getNextPoint(pointsOnBot, i);

            double edgeX = point1.x - point0.x; //getting the normal of the edge's slope so we can project
            double edgeY = point1.y - point0.y; //the points onto the normal so we can get the mins and maxes and compare them :   )

            Vector2d normal = new Vector2d(-edgeY, edgeX);
            Vector2d unitNormal = new Vector2d(normal.getX()/normal.magnitude(), normal.getY()/normal.magnitude());

            double[] projectedRobot = getProjectedValuesFromList(pointsOnBot, unitNormal);
            double[] projectedBigTriangle = getProjectedValuesFromList(pointsInBigTriangle, unitNormal);
            double[] projectedSmallTriangle = getProjectedValuesFromList(pointsInSmallTriangle, unitNormal);

            if ((projectedRobot[1] < projectedBigTriangle[0] || projectedBigTriangle[1] < projectedRobot[0]) &&
                (projectedRobot[1] < projectedSmallTriangle[0] || projectedSmallTriangle[1] < projectedRobot[0]))
            {
                return false;
            }


        }

        return true;
    }

    public boolean isInDangerZone(Pose position, Teams team) {
        //there are simpler ways to do this but i wanted to do this cause its cool
        double botX = position.getX();
        double botY = position.getY();
        double botHeading = position.getHeading();
        boolean isColliding;
        Point[] pointsOnBot = rotateSquare(botX, botY, botHeading);


        for (int i =0; i < pointsOnBot.length; i++) {
            Point point0 = pointsOnBot[i];
            Point point1 = getNextPoint(pointsOnBot, i);

            double edgeX = point1.x - point0.x; //getting the normal of the edge's slope so we can project
            double edgeY = point1.y - point0.y; //the points onto the normal so we can get the mins and maxes and compare them :   )

            Vector2d normal = new Vector2d(-edgeY, edgeX);
            Vector2d unitNormal = new Vector2d(normal.getX()/normal.magnitude(), normal.getY()/normal.magnitude());

            double[] projectedRobot = getProjectedValuesFromList(pointsOnBot, unitNormal);
            double[] projectedRedDanger = getProjectedValuesFromList(pointsInDangerZoneRed, unitNormal);
            double[] projectedBlueDanger = getProjectedValuesFromList(pointsInDangerZoneBlue, unitNormal);

            if ((projectedRobot[1] < projectedRedDanger[0] || projectedRedDanger[1] < projectedRobot[0]) && team == Teams.BLUE) {
                return false;
            }
            if ((projectedRobot[1] < projectedBlueDanger[0] || projectedBlueDanger[1] < projectedRobot[0]) && team == Teams.RED) {
                return false;
            }


        }

        return true;
    }

    Point getNextPoint(Point[] pointsList, int i) {
        Point nextPoint;
        if (i == pointsList.length-1) {
            nextPoint = pointsList[0];
        } else {
            nextPoint = pointsList[i+1];
        }
        return nextPoint;
    }

    double[] getProjectedValuesFromList(Point[] listOfPointsToProject, Vector2d vectorToProject) {
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;

        for (int i = 0; i < listOfPointsToProject.length; i++) {
            Point pointToTest = listOfPointsToProject[i];
            double projectedValue = vectorToProject.dot(new Vector2d(pointToTest.x, pointToTest.y));

            if (projectedValue < min) min = projectedValue;
            if (projectedValue > max) max = projectedValue;
        }
        return new double[]{min, max};
    }

    public Pose getPositionFromAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double xToReturn = 0;
        double yToReturn = 0;
        double headingToReturn = 0;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 24 || detection.id == 20) {
                yToReturn = (-detection.robotPose.getPosition().x - 4) + 72;
                xToReturn = (detection.robotPose.getPosition().y - 4) + 72;
                headingToReturn = Math.toRadians(detection.robotPose.getOrientation().getYaw());
            }
        }

        return new Pose(xToReturn, yToReturn, headingToReturn);
    }

    public void telemetryAprilTag(Telemetry telemetry) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {

            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addData("field x", detection.robotPose.getPosition().x);
                telemetry.addData("field y", detection.robotPose.getPosition().y);
                telemetry.addData("yaw", detection.robotPose.getOrientation().getYaw());

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }




}
