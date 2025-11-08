package org.firstinspires.ftc.teamcode.teleop;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class BasicDrive extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    static boolean headingLock = false;
    double targetHeading = Math.toRadians(90);
    double targetVelocity = 380;
    double targetPitch = 195;
    int lockTicks = 100;
    PIDFController headingPIDController = new PIDFController(new PIDFCoefficients(0, 0, 0, 0));
    double headingError = 0;
    Hardware.Teams team;
    Hardware robot;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        if (robot.endPositionBlackboard.get() != null) {
            Object endPosFromAuto = robot.endPositionBlackboard.get();
            follower.setStartingPose((Pose) endPosFromAuto);
        } else {
            follower.setStartingPose(new Pose());
        }
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new Hardware(hardwareMap);
        headingPIDController.setCoefficients(follower.getConstants().coefficientsHeadingPIDF);

        if (robot.teamBlackboard.get() == Hardware.Teams.RED) {
            team = Hardware.Teams.RED;
        } else if (robot.teamBlackboard.get() == Hardware.Teams.BLUE) {
            team = Hardware.Teams.BLUE;
        }

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(0, 0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        robot.shooterRight.stop();
        robot.shooterLeft.stop();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {

            if (headingLock) {
                Pose positionToPoint = new Pose(0, 0);
                if (team == Hardware.Teams.RED) {
                    positionToPoint = new Pose(58, -58);
                } else if (team == Hardware.Teams.BLUE) {
                    positionToPoint = new Pose(58, 58);
                }
                Pose currentPosition = follower.getPose();
                targetHeading = (-1 * Math.atan(
                        (positionToPoint.getX() - (currentPosition.getX()-72))
                                /
                                (positionToPoint.getY() - (currentPosition.getY()-72))
                ));
                if (positionToPoint.getY() < 0) {
                    targetHeading = targetHeading - (Math.PI / 2);
                } else {
                    targetHeading = targetHeading + (Math.PI / 2);
                }

                headingError = targetHeading - follower.getHeading();
                headingPIDController.setCoefficients(follower.constants.coefficientsHeadingPIDF);
                headingPIDController.updateError(headingError);

                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        headingPIDController.run(),
                        true
                );
            } else {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x * 0.7,
                        true
                );
            }
        }

        if (gamepad1.leftBumperWasPressed()) {
            robot.intakeFront.start();
            robot.intakeBack.start();
            robot.chuteRight.start();
            robot.chuteLeft.start();
        }
        if (gamepad1.leftBumperWasReleased()) {
            robot.intakeFront.stop();
            robot.intakeBack.stop();
            robot.chuteRight.stop();
            robot.chuteLeft.stop();
        }

        Hardware.ArtifactType detectedArtifactFront = robot.intakeFront.colorSensor.detectColor();
        Hardware.ArtifactType detectedArtifactBack = robot.intakeBack.colorSensor.detectColor();
        robot.intakeFront.colorSensor.trackColor(detectedArtifactFront);
        robot.intakeBack.colorSensor.trackColor(detectedArtifactBack);

        if (detectedArtifactFront != Hardware.ArtifactType.NONE) { //replace this later for a smarter system
            robot.sorter.updateServo(detectedArtifactFront);
        } else if (detectedArtifactBack != Hardware.ArtifactType.NONE) {

            if (detectedArtifactBack == Hardware.ArtifactType.GREEN) { //invert back updating (i should just make this a function)
                robot.sorter.updateServo(Hardware.ArtifactType.PURPLE);
            } else if (detectedArtifactBack == Hardware.ArtifactType.PURPLE) {
                robot.sorter.updateServo(Hardware.ArtifactType.GREEN);
            }

        } else {
            robot.sorter.updateServo(Hardware.ArtifactType.NONE);
        }
        if (gamepad1.rightBumperWasPressed()) {
            if (robot.teamBlackboard.get() == Hardware.Teams.RED) {
                robot.teamBlackboard.set(Hardware.Teams.BLUE);
                team = Hardware.Teams.BLUE;
                gamepad1.rumble(50);
            } else {
                robot.teamBlackboard.set(Hardware.Teams.RED);
                team = Hardware.Teams.RED;
                gamepad1.rumble(50);
            }
        }

        if (gamepad1.yWasPressed()) {
            follower.setPose(follower.getPose().withHeading(0));
        }



        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //if (gamepad1.xWasPressed()) {
        //    headingLock = !headingLock;
        //}


            if (gamepad1.right_trigger > 0) {
                robot.shooterRight.setLauncherVelocity(targetVelocity);
                if (robot.shooterRight.isLauncherWithinVelocity()) {
                    robot.lock.open();
                    robot.chuteRight.start();
                } else {
                    robot.lock.close();
                    robot.chuteRight.stop();
                }
            } else {
                robot.shooterRight.stop();
                if (!(gamepad1.left_trigger > 0)) {
                    robot.lock.close();
                }
                if (!(gamepad1.left_bumper)) {
                    robot.chuteRight.stop();
                }
            }

            if (gamepad1.left_trigger > 0) {
                robot.shooterLeft.setLauncherVelocity(targetVelocity);
                if (robot.shooterLeft.isLauncherWithinVelocity()) {
                    robot.lock.open();
                    robot.chuteLeft.start();
                } else {
                    robot.lock.close();
                    robot.chuteLeft.stop();
                }
            } else {
                robot.shooterLeft.stop();
                if (!(gamepad1.right_trigger > 0)) {
                    robot.lock.close();
                }
                if (!(gamepad1.left_bumper)) {
                    robot.chuteLeft.stop();
                }
            }

        robot.shooterRight.keepLauncherAtVelocity();
        robot.shooterLeft.keepLauncherAtVelocity();
        if (gamepad1.dpad_up) {
            robot.lock.open();
        }
        if (gamepad1.dpad_right) {
            if (targetPitch < 200) {
                targetPitch = targetPitch + 5;
            }
            robot.shooterRight.updatePitch(targetPitch);
            robot.shooterLeft.updatePitch(targetPitch);
        }
        if (gamepad1.dpad_left) {
            if (targetPitch > 0) {
                targetPitch = targetPitch - 5;
            }
            robot.shooterRight.updatePitch(targetPitch);
            robot.shooterLeft.updatePitch(targetPitch);
        }


        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("target velocity shooter", targetVelocity);
        telemetryM.debug("velocity shooter", robot.shooterRight.getLauncherVelocity());
        telemetryM.debug("power shooter", robot.shooterLeft.launcherMotor.get());
        telemetryM.debug("are we at velocity:", robot.shooterLeft.isLauncherAtVelocity());
        telemetryM.debug("pitch", robot.shooterLeft.getPitchAngle());
        telemetryM.debug("seq", robot.sequence);
        telemetryM.debug("detected front", detectedArtifactFront);
        telemetryM.debug("detected back", detectedArtifactBack);
        telemetryM.debug("detected front", robot.intakeFront.colorSensor.getCamera());
        telemetryM.debug("detected back", robot.intakeBack.colorSensor.getCamera());
        telemetryM.debug("automatedDrive", automatedDrive);
    }

    @Override
    public void stop() {
        robot.endPositionBlackboard.set(follower.getPose());
    }
}