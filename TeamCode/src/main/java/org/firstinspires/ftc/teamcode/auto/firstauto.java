package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.path_noGate.BluePath;
import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "L1 auto", group = "auto")
public class firstauto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    public Hardware robot;
    public BluePath pathMaster;

    public void buildPaths() {
        pathMaster = new BluePath(follower);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(pathMaster.StartToShoot);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    robot.intakeFront.start();
                    robot.intakeBack.start();
                    robot.chuteRight.start();
                    robot.chuteLeft.start();
                    follower.followPath(pathMaster.ShootToLOne, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pathMaster.LOne, 0.5, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    robot.intakeFront.stop();
                    robot.intakeBack.stop();
                    robot.chuteRight.stop();
                    robot.chuteLeft.stop();
                    follower.followPath(pathMaster.LOneToShoot, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    robot.intakeFront.start();
                    robot.intakeBack.start();
                    robot.chuteRight.start();
                    robot.chuteLeft.start();
                    follower.followPath(pathMaster.ShootToLTwo, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(pathMaster.LTwo, 0.5, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    robot.intakeFront.stop();
                    robot.intakeBack.stop();
                    robot.chuteRight.stop();
                    robot.chuteLeft.stop();
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
                    follower.followPath(pathMaster.ShootToLThree, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(pathMaster.LThree, 0.5, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    robot.intakeFront.stop();
                    robot.intakeBack.stop();
                    robot.chuteRight.stop();
                    robot.chuteLeft.stop();
                    follower.followPath(pathMaster.LThreeToFarShoot, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }

        if (robot.intakeFront.isRotating) {
            Hardware.ArtifactType detectedFront = robot.intakeFront.colorSensor.detectColor();
            Hardware.ArtifactType detectedBack = robot.intakeBack.colorSensor.detectColor();
            telemetry.addData("dfront", detectedFront);
            telemetry.addData("dback", detectedBack);
            if (detectedFront != Hardware.ArtifactType.NONE) {
                robot.sorter.updateServo(detectedFront, false);
            } else if (detectedBack != Hardware.ArtifactType.NONE) {
                robot.sorter.updateServo(detectedBack, true);
            }
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
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        robot = new Hardware(hardwareMap);

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
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }

}