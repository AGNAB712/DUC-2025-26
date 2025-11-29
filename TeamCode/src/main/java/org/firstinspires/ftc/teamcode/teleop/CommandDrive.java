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
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.lib.Commands;
import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class CommandDrive extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    double pitchAngle = 0;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    static boolean headingLock = false;
    double targetHeading = Math.toRadians(90);
    double atVelTicks = 0;
    static double targetVelocity = 1500;
    static double targetAngle = 1500;
    PIDFController headingPIDController = new PIDFController(new PIDFCoefficients(0, 0, 0, 0));
    double headingError = 0;
    double headingOffset = 0;
    double velocityError = 0;
    double thePowerForTheLauncher = 0;
    boolean isIntaking = false;
    Hardware.Teams team = Hardware.Teams.BLUE;
    Hardware robot;
    Hardware.VelocityLUT velLUT = new Hardware.VelocityLUT();
    GamepadEx driverGamepad;
    private Supplier<PathChain> toRedBase;
    private Supplier<PathChain> toBlueBase;
    boolean rightShooterKeepAtVelocity = false;
    boolean leftShooterKeepAtVelocity = false;
    double lastVelocityLeft = 0;
    double lastVelocityRight = 0;
    double sorterTargetAngle = 0;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        if (robot.endPositionBlackboard.get() != null) {
            Object endPosFromAuto = robot.endPositionBlackboard.get();
            follower.setStartingPose((Pose) endPosFromAuto);
        } else {
            follower.setStartingPose(new Pose());
        }
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        headingPIDController.setCoefficients(follower.getConstants().coefficientsHeadingPIDF);

        if (robot.teamBlackboard.get() == Hardware.Teams.RED) {
            team = Hardware.Teams.RED;
        } else if (robot.teamBlackboard.get() == Hardware.Teams.BLUE) {
            team = Hardware.Teams.BLUE;
        }

        Commands commandsList = new Commands();
        driverGamepad = new GamepadEx(gamepad1);

        toRedBase = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(36, 33.5))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();
        toBlueBase = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(104, 33.5))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();

        Button intakeButton = new GamepadButton(
                driverGamepad, GamepadKeys.Button.A
        ).toggleWhenPressed(commandsList.new RunIntake(robot.chuteRight, robot.chuteLeft, robot.intakeFront, robot.intakeBack, robot.lock));
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
                if (team == Hardware.Teams.BLUE) {
                    positionToPoint = new Pose(10, 130);
                } else if (team == Hardware.Teams.RED) {
                    positionToPoint = new Pose(130, 130);
                }
                Pose currentPosition = follower.getPose();
                targetHeading = (-1 * Math.atan(
                        (positionToPoint.getX() - currentPosition.getX())
                                /
                                (positionToPoint.getY() - currentPosition.getY())
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
                        false,
                        headingOffset
                );
            } else {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x * 0.7,
                        false,
                        headingOffset
                );
            }
        }

        if (gamepad1.yWasPressed()) {
            headingOffset = follower.getHeading();
        }
        if (gamepad1.aWasPressed()) {
            isIntaking = !isIntaking;
        }
        if (isIntaking) {
            if (gamepad1.dpad_down) {
                robot.chuteLeft.reverse();
                robot.chuteRight.reverse();
                robot.intakeFront.reverse();
                robot.intakeBack.reverse();
            } else {
                robot.chuteLeft.start();
                robot.chuteRight.start();
                robot.intakeFront.start();
                robot.intakeBack.start();
            }
        } else {
            robot.chuteLeft.stop();
            robot.chuteRight.stop();
            robot.intakeFront.stop();
            robot.intakeBack.stop();
        }
        if (gamepad1.dpadLeftWasPressed()) {
            sorterTargetAngle = sorterTargetAngle + 5;
            robot.lock.chuteLock.turnToAngle(sorterTargetAngle);
        } else if (gamepad1.dpadRightWasPressed()) {
            sorterTargetAngle = sorterTargetAngle - 5;
            robot.lock.chuteLock.turnToAngle(sorterTargetAngle);
        }

        if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
            double[] velLutOutput = velLUT.get(Hardware.distanceToGoal(team, follower.getPose()));
            targetVelocity = velLutOutput[0];
            targetAngle = velLutOutput[1];
        }

        if (gamepad1.right_trigger > 0) {
            shoot(targetVelocity, targetAngle, false);
        } else {
            if (rightShooterKeepAtVelocity) {
                keepShooterAtVelocity(robot.shooterRight, 1000);
            } else {
                robot.shooterRight.launcherMotor.set(0);
            }
        }

        if (gamepad1.left_trigger > 0) {
            shoot(targetVelocity, targetAngle, true);
        } else {
            if (leftShooterKeepAtVelocity) {
                keepShooterAtVelocity(robot.shooterLeft, 1000);
            } else {
                robot.shooterLeft.launcherMotor.set(0);
            }
        }
        if (gamepad1.leftBumperWasPressed()) {
            leftShooterKeepAtVelocity = !leftShooterKeepAtVelocity;
        }
        if (gamepad1.rightBumperWasPressed()) {
            rightShooterKeepAtVelocity = !rightShooterKeepAtVelocity;
        }

        if (gamepad1.rightStickButtonWasPressed()) {
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
        if (gamepad1.leftStickButtonWasPressed()) {
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
        }

        if (gamepad1.xWasPressed()) {
            headingLock = !headingLock;
        }

        if (gamepad1.startWasPressed()) {
            if (team == Hardware.Teams.RED) {
                follower.followPath(toRedBase.get());
            } else {
                follower.followPath(toBlueBase.get());
            }
            automatedDrive = true;
        }
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (gamepad1.dpad_left) {
            automatedDrive = true;
        }
        //robot.shooterRight.setPitchAngle(pitchAngle, true);
        //robot.shooterLeft.setPitchAngle(pitchAngle, false);

        if (robot.intakeFront.isRotating || gamepad1.dpad_up) {
            Hardware.ArtifactType detectedFront = robot.intakeFront.colorSensor.detectColor();
            Hardware.ArtifactType detectedBack = robot.intakeBack.colorSensor.detectColor();
            telemetryM.addData("dfront", detectedFront);
            telemetryM.addData("dback", detectedBack);
            if (detectedFront != Hardware.ArtifactType.NONE) {
                robot.sorter.updateServo(detectedFront, false);
            } else if (detectedBack != Hardware.ArtifactType.NONE) {
                robot.sorter.updateServo(detectedBack, true);
            } else {
                robot.sorter.updateServo(Hardware.ArtifactType.NONE, false);
            }
        }

        CommandScheduler.getInstance().run();

        if (robot.isInDangerZone(follower.getPose(), team)) {
            gamepad1.rumble(100);
        }





        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        robot.telemetryAprilTag(telemetry);
        telemetryM.addData("team", team);
        telemetryM.addData("launcherPower right", robot.shooterRight.launcherMotor.getCorrectedVelocity());
        telemetryM.addData("launcherPower left", robot.shooterLeft.launcherMotor.getCorrectedVelocity());
        telemetryM.addData("launcher target vel", targetVelocity);
        telemetryM.addData("sorter servo angle", sorterTargetAngle);
        telemetryM.addData("distance to team goal", Hardware.distanceToGoal(team, follower.getPose()));
        telemetryM.addData("is in shooting area?", robot.isInTriangle(follower.getPose()));
        telemetryM.addData("is in danger zone?", robot.isInDangerZone(follower.getPose(), team));

        telemetryM.update(telemetry);
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

        shooter.setPitchAngle(targetAngle, isLeftSide);
        keepShooterAtVelocity(shooter, targetVelocity);


        if (shooter.launcherMotor.getCorrectedVelocity() < (isLeftSide ? lastVelocityLeft : lastVelocityRight)) {
            robot.lock.close();
            chute.stop();
            gamepad1.rumble(500);
        }

        if (shooter.launcherMotor.getCorrectedVelocity() > targetPosition - 30) {
            atVelTicks++;
            if (atVelTicks > 10) {
                chute.start();
                robot.lock.open();
                if (isLeftSide) {
                    gamepad1.setLedColor(0, 1, 0, 500);
                } else {
                    gamepad1.setLedColor(0.5, 0, 0.5, 500);
                }
            }
        } else {
            atVelTicks = 0;
        }

        lastVelocityLeft = robot.shooterLeft.launcherMotor.getCorrectedVelocity();
        lastVelocityRight =robot.shooterRight.launcherMotor.getCorrectedVelocity();
    }
    void keepShooterAtVelocity(Hardware.Shooter shooter, double targetPosition) {
        if (shooter.launcherMotor.getCorrectedVelocity() > targetPosition) {
            shooter.launcherMotor.set(0.001);
        } else {
            shooter.launcherMotor.set(1);
        }
    }
}