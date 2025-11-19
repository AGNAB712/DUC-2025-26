package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
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

import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


@Configurable
@TeleOp(name = "Teleop", group = "Teleop")
public class Drive extends OpMode {

    @IgnoreConfigurable
    static public TelemetryManager telemetryM;
    @IgnoreConfigurable
    Hardware robot;
    @IgnoreConfigurable
    Follower follower;

    GamepadEx gamepadDrive;
    GamepadEx gamepadSubsystem;
    Hardware.Teams team;

    static boolean traveling = false;
    static boolean headingLock = false;
    double headingError = 0;
    double targetHeading = Math.toRadians(90);
    PIDFController headingPIDController = new PIDFController(new PIDFCoefficients(0, 0, 0, 0));
    private Supplier<PathChain> pathChain;

    @Override
    public void init() {
        gamepadDrive = new GamepadEx(gamepad1);
        gamepadSubsystem = new GamepadEx(gamepad2);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72));
        robot = new Hardware(hardwareMap);

        if (robot.teamBlackboard.get() == Hardware.Teams.RED) {
            team = Hardware.Teams.RED;
        } else if (robot.teamBlackboard.get() == Hardware.Teams.BLUE) {
            team = Hardware.Teams.BLUE;
        }

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(72, 72))))
                .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(45))
                .build();

    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        /*if (!traveling) {
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
                        gamepadDrive.getLeftY(),
                        -gamepadDrive.getLeftX(),
                        headingPIDController.run(),
                        false
                );
            } else {
                follower.setTeleOpDrive(
                        gamepadDrive.getLeftY(),
                        -gamepadDrive.getLeftX(),
                        -gamepadDrive.getRightX() * 0.7,
                        false
                );
            }



        }*/
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );




        if (gamepadDrive.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (robot.teamBlackboard.get() == Hardware.Teams.RED) {
                robot.teamBlackboard.set(Hardware.Teams.BLUE);
                team = Hardware.Teams.BLUE;
            } else {
                robot.teamBlackboard.set(Hardware.Teams.RED);
                team = Hardware.Teams.RED;
            }
        }
        if (gamepadDrive.wasJustPressed(GamepadKeys.Button.A)) {
            follower.followPath(pathChain.get());
            traveling = true;
        }
        if (gamepadDrive.wasJustPressed(GamepadKeys.Button.B) && traveling || !follower.isBusy()) {
            follower.startTeleopDrive();
            traveling = false;
        }
        if (gamepadDrive.wasJustPressed(GamepadKeys.Button.X)) {
            headingLock = !headingLock;
        }

        if (gamepadDrive.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            robot.lock.open();
        }
        if (gamepadDrive.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            robot.lock.close();
        }
        if (gamepadDrive.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.intakeFront.start();
            robot.intakeBack.start();
            robot.chuteRight.start();
            robot.chuteLeft.start();
        }
        if (gamepadDrive.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.intakeFront.stop();
            robot.intakeBack.stop();
            robot.chuteRight.stop();
            robot.chuteLeft.stop();
        }
        if (gamepadDrive.wasJustPressed(GamepadKeys.Button.Y)) {
            robot.shooterRight.setPitchAngle(17, true);
        }
        if (gamepadDrive.wasJustReleased(GamepadKeys.Button.Y)) {
            robot.shooterRight.setPitchAngle(0, true);
        }



        gamepadSubsystem.readButtons();
        gamepadDrive.readButtons();

        telemetryM.debug("team:" + robot.teamBlackboard.get());
        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        //telemetryM.debug("Color:" + detectedArtifact);
        telemetryM.debug("current sequence:" + robot.getCurrentArtifacts());
        telemetryM.debug("velocity x:" + follower.getVelocity().getXComponent());
        telemetryM.debug("velocity y:" + follower.getVelocity().getYComponent());
        telemetryM.debug("heading error: " + headingError);
        //robot.shooter.yawServo.update();
        //telemetryM.debug("total angle:" + robot.shooter.yawServo.showTelemetryData()[0]);
        //telemetryM.debug("rots:" + robot.shooter.yawServo.showTelemetryData()[1]);
        //telemetryM.debug("distance:" + robot.shooter.yawServo.showTelemetryData()[2]);
        //telemetryM.debug("direction:" + robot.shooter.yawServo.showTelemetryData()[3]);
        //telemetryM.update(telemetry);



        //drawCurrentAndHistory();
    }
}