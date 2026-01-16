package org.firstinspires.ftc.teamcode.auto.path_noGate;

import static org.firstinspires.ftc.teamcode.auto.path_noGate.Poses.shootPosition;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "RedBasic", group = "nogate")
@Configurable
public class RedBasic extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer;
    private ElapsedTime opmodeTimer;

    private int pathState;
    public Hardware robot;
    public RedPath pathMaster;
    static double targetVelocity = 1500;
    double lastVelocityLeft = 0;
    double lastVelocityRight = 0;
    double atVelTicks = 0;
    boolean leftIsShooting = false;
    boolean rightIsShooting = false;
    boolean leftHasShot = false;
    boolean rightHasShot = false;
    static double velocityForMidShooting = 1200;
    int rightTicks = 0;
    int leftTicks = 0;
    int shootToResetTo = 0;
    int timesHasShot = 0;
    PIDFController shooterVelocityPIDController;
    boolean sorterGoesCrazy = false;
    public List<Hardware.ArtifactType> sequence = new ArrayList<>();
    static PIDFCoefficients shooterVelPIDCoeffs = new PIDFCoefficients(0.03, 0.0, 0.00001, 0);
    Hardware.VelocityLUT velLUT = new Hardware.VelocityLUT();

    public void buildPaths() {
        pathMaster = new RedPath(follower);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pathMaster.StartToShoot);
                sequence.add(Hardware.ArtifactType.GREEN);
                sequence.add(Hardware.ArtifactType.PURPLE);
                sequence.add(Hardware.ArtifactType.PURPLE);
                setPathState(1);
                break;
            case 1:
                keepShooterAtVelocity(robot.shooterLeft, velocityForMidShooting);
                keepShooterAtVelocity(robot.shooterRight, velocityForMidShooting);
                if (!follower.isBusy()) {
                    Pose newPosition = robot.getPositionFromAprilTag();
                    shootToResetTo = 3;
                    if (pathTimer.getElapsedTimeSeconds() > 5) {
                        setPathState(100);
                    }
                    if ((newPosition.getX() != 0 && newPosition.getY() != 0)) {
                        follower.setPose(newPosition);
                        velocityForMidShooting = velLUT.get(Hardware.distanceToGoal(Hardware.Teams.RED, follower.getPose()))[0];
                        setPathState(100);
                    }
                }
                break;
            case 2:
                keepShooterAtVelocity(robot.shooterLeft, velocityForMidShooting);
                keepShooterAtVelocity(robot.shooterRight, velocityForMidShooting);
                if (!follower.isBusy()) {
                    follower.followPath(pathMaster.TagToShoot);
                    shootToResetTo = 3;
                    setPathState(100);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    robot.intakeFront.start();
                    robot.intakeBack.start();
                    robot.chuteRight.start();
                    robot.chuteLeft.start();
                    robot.lock.close();
                    follower.followPath(follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(follower.getPose(), new Pose(45.360, 83.520).mirror())
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), 0)
                            .build(), true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pathMaster.LOne, 0.6, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(pathMaster.LTwoToShoot, true);
                    shootToResetTo = 6;
                    sequence = new ArrayList<>();
                    sequence.add(Hardware.ArtifactType.PURPLE);
                    sequence.add(Hardware.ArtifactType.PURPLE);
                    sequence.add(Hardware.ArtifactType.PURPLE);
                    setPathState(100);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(pathMaster.ShootToLeave, true);
                    setPathState(6);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;


            case 104:
                if (!follower.isBusy()) {
                    shoot(velocityForMidShooting, 17, false);
                    shoot(velocityForMidShooting, 17, true);
                    if (leftHasShot && rightHasShot) {
                        leftHasShot = false;
                        rightHasShot = false;
                        lastVelocityLeft = 0;
                        lastVelocityRight = 0;
                        setPathState(105);
                    }
                }
                break;
            case 105:
                if (!follower.isBusy()) {
                    robot.shooterLeft.stop();
                    shoot(velocityForMidShooting, 17, false);
                    if (rightHasShot) {
                        rightHasShot = false;
                        lastVelocityRight = 0;
                        setPathState(shootToResetTo);
                    }
                }
            case 100:
                if (!follower.isBusy()) { //make this a function later PLEASE.....
                    if (opmodeTimer.milliseconds() > 28000) {
                        setPathState(6);
                    }
                    if (sequence.get(0) == Hardware.ArtifactType.GREEN) {
                        keepShooterAtVelocity(robot.shooterRight, velocityForMidShooting);
                        shoot(velocityForMidShooting, 17, true);
                        if (leftHasShot) {
                            leftHasShot = false;
                            lastVelocityLeft = 0;
                            setPathState(101);
                        }
                    } else if (sequence.get(0) == Hardware.ArtifactType.PURPLE) {
                        keepShooterAtVelocity(robot.shooterLeft, velocityForMidShooting);
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
                    if (opmodeTimer.milliseconds() > 28000) {
                        setPathState(6);
                    }
                    if (sequence.get(1) == Hardware.ArtifactType.GREEN) {
                        keepShooterAtVelocity(robot.shooterRight, velocityForMidShooting);
                        shoot(velocityForMidShooting, 17, true);
                        if (leftHasShot) {
                            leftHasShot = false;
                            lastVelocityLeft = 0;
                            setPathState(102);
                        }
                    } else {
                        keepShooterAtVelocity(robot.shooterLeft, velocityForMidShooting);
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
                    if (opmodeTimer.milliseconds() > 28000) {
                        setPathState(6);
                    }
                    if (sequence.get(2) == Hardware.ArtifactType.GREEN) {
                        keepShooterAtVelocity(robot.shooterRight, velocityForMidShooting);
                        shoot(velocityForMidShooting, 17, true);
                        if (leftHasShot) {
                            leftHasShot = false;
                            lastVelocityLeft = 0;
                            setPathState(shootToResetTo);
                        }
                    } else {
                        keepShooterAtVelocity(robot.shooterLeft, velocityForMidShooting);
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

        if (opmodeTimer.seconds() >= 29) {
            robot.endPositionBlackboard.set(follower.getPose());
            robot.teamBlackboard.set(Hardware.Teams.RED);
        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.addData("shooting left", leftIsShooting);
        telemetry.addData("shooting right", rightIsShooting);
        telemetry.addData("launcherPower right", robot.shooterRight.launcherMotor.getCorrectedVelocity());
        telemetry.addData("launcherPower left", robot.shooterLeft.launcherMotor.getCorrectedVelocity());
        telemetry.addData("leftTicks", leftTicks);
        telemetry.addData("rightTicks", rightTicks);
        telemetry.addData("has shot left", leftHasShot);
        telemetry.addData("has shot right", rightHasShot);

        telemetry.addData("timer", pathTimer.getElapsedTimeSeconds());

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

        shooterVelocityPIDController = new PIDFController(shooterVelPIDCoeffs);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(Poses.startPosition.mirror());

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
        robot.teamBlackboard.set(Hardware.Teams.RED);
    }

    @Override
    public void stop() {

        robot.endPositionBlackboard.set(follower.getPose());
        robot.teamBlackboard.set(Hardware.Teams.RED);
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

        keepShooterAtVelocity(shooter, targetPosition);

        if (shooter.launcherMotor.getCorrectedVelocity() > targetPosition - 20 &&
            shooter.launcherMotor.getCorrectedVelocity() < targetPosition + 20 &&
            !(isLeftSide ? leftIsShooting : rightIsShooting)) {
            if(isLeftSide) {leftTicks++;} else {rightTicks++;}
        } else {
            //if(isLeftSide) {leftTicks=0;} else {rightTicks=0;}
        }

        if (isLeftSide ? leftTicks > 1 : rightTicks > 1) {
            if (isLeftSide) {
                leftIsShooting = true;
                gamepad1.setLedColor(0, 1, 0, 500);
                leftTicks = 0;
            } else {
                rightIsShooting = true;
                gamepad1.setLedColor(0.5, 0, 0.5, 500);
                rightTicks = 0;
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
        double error = targetPosition - shooter.launcherMotor.getCorrectedVelocity();
        shooterVelocityPIDController.updateError(error);
        double power = shooterVelocityPIDController.run();
        if (power < 0) {
            power = 0;
        }
        double powerClamped = Range.clip(power, 0, 1);
        telemetry.addData("power", power);
        telemetry.addData("error", error);
        shooter.launcherMotor.set(powerClamped);
    }

}