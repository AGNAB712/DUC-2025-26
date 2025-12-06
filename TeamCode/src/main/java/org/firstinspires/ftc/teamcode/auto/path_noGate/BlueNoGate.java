package org.firstinspires.ftc.teamcode.auto.path_noGate;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BlueFarNoGate", group = "nogate")
@Disabled
public class BlueNoGate extends OpMode {

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
    double velocityForMidShooting = 1525;
    int shootToResetTo = 0;
    int timesHasShot = 0;
    boolean sorterGoesCrazy = true;
    ElapsedTime shotTimer = new ElapsedTime(100000000);

    public void buildPaths() {
        pathMaster = new BluePath(follower);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pathMaster.StartToShoot);
                shootToResetTo = 1;
                setPathState(100);
                break;
            case 1:
                if (!follower.isBusy()) {
                    robot.intakeFront.start();
                    robot.intakeBack.start();
                    robot.chuteRight.start();
                    robot.chuteLeft.start();
                    robot.lock.close();
                    follower.followPath(pathMaster.ShootToLOne, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pathMaster.LOne, 0.6, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(pathMaster.LOneToShoot, true);
                    shootToResetTo = 4;
                    setPathState(100);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    robot.intakeFront.start();
                    robot.intakeBack.start();
                    robot.chuteRight.start();
                    robot.chuteLeft.start();
                    robot.lock.close();
                    follower.followPath(pathMaster.ShootToLTwo, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(pathMaster.LTwo, 0.6, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {

                    follower.followPath(pathMaster.LTwoToShoot, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    robot.intakeFront.start();
                    robot.intakeBack.start();
                    robot.chuteRight.start();
                    robot.chuteLeft.start();
                    robot.lock.close();
                    follower.followPath(pathMaster.ShootToLThree, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(pathMaster.LThree, 0.6, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    robot.intakeFront.stop();
                    robot.intakeBack.stop();
                    robot.chuteRight.stop();
                    robot.chuteLeft.stop();
                    follower.followPath(pathMaster.LThreeToCloseShoot, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;



            case 100:
                if (!follower.isBusy()) {
                    shoot(velocityForMidShooting, 0, true);
                    shoot(velocityForMidShooting, 0, false);
                    if (leftHasShot) {
                        leftHasShot = false;
                        timesHasShot++;
                    }
                    if (rightHasShot) {
                        rightHasShot = false;
                        timesHasShot++;
                    }
                    if (timesHasShot > 2 || pathTimer.getElapsedTimeSeconds() > 6) {
                        timesHasShot = 0;
                        setPathState(shootToResetTo);
                    }
                }
                break;
            case 101:
                if (!follower.isBusy()) {
                    shoot(velocityForMidShooting, 0, false);
                    if (rightHasShot) {
                        rightHasShot = false;
                        setPathState(shootToResetTo);
                    }
                }
                break;
        }

            /*Hardware.ArtifactType detectedFront = robot.intakeFront.colorSensor.detectColor();
            telemetry.addData("dfront", detectedFront);
            if (detectedFront != Hardware.ArtifactType.NONE) {
                robot.sorter.updateServo(detectedFront, false);
            }*/
        if (sorterGoesCrazy) {
            if (Math.floor(opmodeTimer.milliseconds()/1000) % 2 == 0) {
                if (robot.sorter.isInGreen) {
                    robot.sorter.purple(false);
                } else {
                    robot.sorter.green(false);
                }
            }


        }

        if (pathState < 100) {
            keepShooterAtVelocity(robot.shooterLeft, velocityForMidShooting);
            keepShooterAtVelocity(robot.shooterRight, velocityForMidShooting);
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

        telemetry.addData("times", timesHasShot);

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
        follower.setStartingPose(new Pose(31, 128, Math.toRadians(90)));

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
                shooter.pitchServo.setPosition(0.1);
            } else {
                shooter.pitchServo.setPosition(0.65);
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