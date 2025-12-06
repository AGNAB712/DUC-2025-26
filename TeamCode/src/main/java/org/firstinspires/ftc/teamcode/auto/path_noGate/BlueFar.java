package org.firstinspires.ftc.teamcode.auto.path_noGate;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BlueFar", group = "far")
@Configurable
public class BlueFar extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer;
    private ElapsedTime opmodeTimer;

    private int pathState;
    public Hardware robot;
    public BluePath pathMaster;
    static double targetVelocity = 1500;
    double lastVelocityLeft = 0;
    double lastVelocityRight = 0;
    double atVelTicks = 0;
    boolean leftIsShooting = false;
    boolean rightIsShooting = false;
    boolean leftHasShot = false;
    boolean rightHasShot = false;
    static double velocityForMidShooting = 1850;
    int shootToResetTo = 0;
    int timesHasShot = 0;
    static int delaySeconds = 5;
    boolean sorterGoesCrazy = false;
    ElapsedTime shotTimer = new ElapsedTime(100000000);
    public List<Hardware.ArtifactType> sequence = new ArrayList<>();

    public void buildPaths() {
        pathMaster = new BluePath(follower);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                List<AprilTagDetection> currentDetections = robot.aprilTag.getDetections();

                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == 21) { //once again.... make this a function... please
                        sequence.add(Hardware.ArtifactType.GREEN);
                        sequence.add(Hardware.ArtifactType.PURPLE);
                        sequence.add(Hardware.ArtifactType.PURPLE);
                        setPathState(4);
                    } else if (detection.id == 22) {
                        sequence.add(Hardware.ArtifactType.PURPLE);
                        sequence.add(Hardware.ArtifactType.GREEN);
                        sequence.add(Hardware.ArtifactType.PURPLE);
                        setPathState(4);
                    } else if (detection.id == 23) {
                        sequence.add(Hardware.ArtifactType.PURPLE);
                        sequence.add(Hardware.ArtifactType.PURPLE);
                        sequence.add(Hardware.ArtifactType.GREEN);
                        setPathState(4);
                    }
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(pathMaster.FarStartToShoot);
                    shootToResetTo = 2;
                    setPathState(100);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pathMaster.FarShootToHP);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            case 4:
                if (opmodeTimer.seconds() >= delaySeconds) {
                    setPathState(1);
                }
                break;




            case 100:
                if (!follower.isBusy()) { //make this a function later PLEASE.....
                    if (sequence.get(0) == Hardware.ArtifactType.GREEN) {
                        shoot(velocityForMidShooting, 17, true);
                        if (leftHasShot) {
                            leftHasShot = false;
                            lastVelocityLeft = 0;
                            setPathState(101);
                        }
                    } else {
                        shoot(velocityForMidShooting, 17, false);
                        if (rightHasShot) {
                            rightHasShot = false;
                            lastVelocityRight = 0;
                            setPathState(101);
                        }
                    }
                }
                break;
            case 101:
                if (!follower.isBusy()) {
                    if (sequence.get(1) == Hardware.ArtifactType.GREEN) {
                        shoot(velocityForMidShooting, 17, true);
                        if (leftHasShot) {
                            leftHasShot = false;
                            lastVelocityLeft = 0;
                            setPathState(102);
                        }
                    } else {
                        shoot(velocityForMidShooting, 17, false);
                        if (rightHasShot) {
                            rightHasShot = false;
                            lastVelocityRight = 0;
                            setPathState(102);
                        }
                    }
                }
                break;
            case 102:
                if (!follower.isBusy()) {
                    if (sequence.get(2) == Hardware.ArtifactType.GREEN) {
                        shoot(velocityForMidShooting, 17, true);
                        if (leftHasShot) {
                            leftHasShot = false;
                            lastVelocityLeft = 0;
                            setPathState(shootToResetTo);
                        }
                    } else {
                        shoot(velocityForMidShooting, 17, false);
                        if (rightHasShot) {
                            rightHasShot = false;
                            lastVelocityRight = 0;
                            setPathState(shootToResetTo);
                        }
                    }
                }
                break;
        }

            /*Hardware.ArtifactType detectedFront = robot.intakeFront.colorSensor.detectColor();
            telemetry.addData("dfront", detectedFront);
            if (detectedFront != Hardware.ArtifactType.NONE) {
                robot.sorter.updateServo(detectedFront, false);
            }*/
        robot.sorter.purple(false);

        if (pathState < 100) {
            //keepShooterAtVelocity(robot.shooterLeft, velocityForMidShooting);
            //keepShooterAtVelocity(robot.shooterRight, velocityForMidShooting);
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.addData("shooting left", leftIsShooting);
        telemetry.addData("shooting right", rightIsShooting);
        telemetry.addData("launcherPower right", robot.shooterRight.launcherMotor.getCorrectedVelocity());
        telemetry.addData("launcherPower left", robot.shooterLeft.launcherMotor.getCorrectedVelocity());
        telemetry.addData("has shot left", leftHasShot);
        telemetry.addData("has shot right", rightHasShot);

        telemetry.addData("timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("seq", sequence);

        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new ElapsedTime();
        opmodeTimer.reset();
        robot = new Hardware(hardwareMap);

        robot.teamBlackboard.set(Hardware.Teams.BLUE);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(Poses.startFarPosition);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(0);
    }

    @Override
    public void stop() {
        robot.endPositionBlackboard.set(follower.getPose());
    }

    void shoot(double targetPosition, double targetAngle, boolean isLeftSide) {
        Hardware.Shooter shooter;
        Hardware.Chute chute;
        if (isLeftSide) {
            shooter = robot.shooterLeft;
            chute = robot.chuteLeft;
        } else {
            shooter = robot.shooterRight;
            chute = robot.chuteRight;
        }

        if (targetAngle < 17) { //up
            if (isLeftSide) {
                shooter.pitchServo.setPosition(1);
            } else {
                shooter.pitchServo.setPosition(0.15);
            }
        } else { //down
            if (isLeftSide) {
                shooter.pitchServo.setPosition(0.5); //0.1
            } else {
                shooter.pitchServo.setPosition(0.65); //0.65
            }
        }

        //shooter.setPitchAngle(targetAngle, isLeftSide);
        keepShooterAtVelocity(shooter, targetPosition);

        if (shooter.launcherMotor.getCorrectedVelocity() > targetPosition - 60 && !(isLeftSide ? leftIsShooting : rightIsShooting)) {
            if (isLeftSide) {
                leftIsShooting = true;
                gamepad1.setLedColor(0, 1, 0, 500);
            } else {
                rightIsShooting = true;
                gamepad1.setLedColor(0.5, 0, 0.5, 500);
            }

        }

        if (isLeftSide ? leftIsShooting : rightIsShooting) {
            chute.start();
            robot.lock.open();

            if (shooter.launcherMotor.getCorrectedVelocity() - (isLeftSide ? lastVelocityLeft : lastVelocityRight) < -100) {
                robot.lock.close();
                chute.stop();
                if (isLeftSide) {
                    leftIsShooting = false;
                    leftHasShot = true;
                } else {
                    rightIsShooting = false;
                    rightHasShot = true;
                }
            }
        } else {
            chute.stop();
        }

        lastVelocityLeft = robot.shooterLeft.launcherMotor.getCorrectedVelocity();
        lastVelocityRight = robot.shooterRight.launcherMotor.getCorrectedVelocity();
    }
    void keepShooterAtVelocity(Hardware.Shooter shooter, double targetPosition) {
        if (shooter.launcherMotor.getCorrectedVelocity() > targetPosition) {
            shooter.launcherMotor.set(0.001);
        } else {
            shooter.launcherMotor.set(1);
        }
    }

}