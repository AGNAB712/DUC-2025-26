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
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.lib.Commands;
import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class ShootDrive extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    double pitchAngle = 0;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    static boolean headingLock = false;
    double targetHeading = Math.toRadians(90);
    double leftAtVelTicks = 0;
    double rightAtVelTicks = 0;
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
    GamepadEx driverGamepad;
    private Supplier<PathChain> toRedBase;
    private Supplier<PathChain> toBlueBase;
    boolean rightShooterKeepAtVelocity = false;
    boolean leftShooterKeepAtVelocity = false;
    double lastVelocityLeft = 0;
    double lastVelocityRight = 0;
    double sorterTargetAngle = 0;
    double maxJumpInShooterVelo = 0;
    boolean leftIsShooting = false;
    boolean rightIsShooting = false;
    boolean manualSorting = true;
    Commands commandsList;
    PIDFController shooterVelocityPIDController;
    static PIDFCoefficients shooterVelPIDCoeffs = new PIDFCoefficients(0.03, 0.0, 0.00001, 0);

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

        commandsList = new Commands();
        driverGamepad = new GamepadEx(gamepad1);

        toRedBase = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(36, 33.5))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();
        toBlueBase = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(104, 33.5))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(90), 0.8))
                .build();

        shooterVelocityPIDController = new PIDFController(shooterVelPIDCoeffs);

        Button intakeButton = new GamepadButton(
                driverGamepad, GamepadKeys.Button.A
        ).toggleWhenPressed(commandsList.new RunIntake(robot.chuteRight, robot.chuteLeft, robot.intakeFront, robot.intakeBack, robot.lock));
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        robot.shooterRight.stop();
        robot.shooterLeft.stop();
        if (robot.teamBlackboard.get() == Hardware.Teams.RED) {
            team = Hardware.Teams.RED;
        } else if (robot.teamBlackboard.get() == Hardware.Teams.BLUE) {
            team = Hardware.Teams.BLUE;
        }
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

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
                if (gamepad1.left_trigger < 0.5 && !robot.lock.isOpen) { //if we are not trying to shoot and the lock is closed
                    robot.chuteLeft.start();
                } else if (!leftIsShooting) { //ok lock is probably open
                    robot.chuteLeft.stop();
                }
                if (gamepad1.right_trigger < 0.5 && !robot.lock.isOpen) {
                    robot.chuteRight.start();
                } else if (!rightIsShooting) {
                    robot.chuteRight.stop();
                }

                robot.intakeFront.start();
                robot.intakeBack.start();

            }
        } else {
            if (!leftIsShooting) {
                robot.chuteLeft.stop();
            }
            if (!rightIsShooting) {
                robot.chuteRight.stop();
            }
            robot.intakeFront.stop();
            robot.intakeBack.stop();
        }

        if (gamepad1.dpadLeftWasPressed()) {
            sorterTargetAngle = sorterTargetAngle + 0.1;
            robot.shooterRight.setPitchAngle(sorterTargetAngle, false);
            robot.shooterLeft.setPitchAngle(sorterTargetAngle, true);
        } else if (gamepad1.dpadRightWasPressed()) {
            sorterTargetAngle = sorterTargetAngle - 0.1;
            robot.shooterRight.setPitchAngle(sorterTargetAngle, false);
            robot.shooterLeft.setPitchAngle(sorterTargetAngle, true);
        }

        if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
            //double[] velLutOutput = velLUT.get(Hardware.distanceToGoal(team, follower.getPose()));
            //targetVelocity = velLutOutput[0];
            //targetAngle = velLutOutput[1];
            if (!gamepad1.b) {
                headingLock = true;
            } else {
                headingLock = false;
            }
        } else {
            headingLock = false;
        }

        if (gamepad1.right_trigger > 0) {
            //shoot(targetVelocity, targetAngle, false);
            keepShooterAtVelocity(robot.shooterLeft, targetVelocity);
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
        //robot.shooterRight.setPitchAngle(pitchAngle, true);
        //robot.shooterLeft.setPitchAngle(pitchAngle, false);

        if (gamepad2.bWasPressed()) {
            manualSorting = !manualSorting;
        }

        if (robot.intakeFront.isRotating && !manualSorting) {
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
        if (manualSorting) {
            if (gamepad2.right_stick_x > 0.25) {
                robot.sorter.green(false);
            } else if (gamepad2.right_stick_x < -0.25) {
                robot.sorter.purple(false);
            } else {
                robot.sorter.neutral();
            }
        }

        CommandScheduler.getInstance().run();

        if (robot.isInDangerZone(follower.getPose(), team)) {
            gamepad1.rumble(100);
        }

        if (gamepad2.left_trigger > 0.25) {
            robot.liftLeft.set(gamepad2.left_stick_y);
            robot.liftRight.set(gamepad2.right_stick_y);
        } else {
            if (gamepad2.dpad_up) {
                robot.liftLeft.set(-1);
                robot.liftRight.set(-1);
            } else if (gamepad2.dpad_down) {
                robot.liftLeft.set(1);
                robot.liftRight.set(1);
            } else {
                robot.liftLeft.set(0);
                robot.liftRight.set(0);
            }
        }



        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        robot.telemetryAprilTag(telemetry);
        telemetryM.addData("team", team);
        telemetryM.addData("launcherPower right", robot.shooterRight.launcherMotor.getCorrectedVelocity());
        telemetryM.addData("launcherPower left", robot.shooterLeft.launcherMotor.getCorrectedVelocity());
        telemetryM.addData("max jump in shooter velocity", maxJumpInShooterVelo);
        telemetryM.addData("launcher target vel", targetVelocity);
        telemetryM.addData("sorter servo angle", sorterTargetAngle);
        telemetryM.addData("distance to team goal", Hardware.distanceToGoal(team, follower.getPose()));
        telemetryM.addData("is in shooting area?", robot.isInTriangle(follower.getPose()));
        telemetryM.addData("shooting left", leftIsShooting);
        telemetryM.addData("shooting right", rightIsShooting);
        telemetryM.addData("pitch right", robot.shooterRight.pitchServo.getPosition());
        telemetryM.addData("pitch left", robot.shooterLeft.pitchServo.getPosition());
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

        if (isLeftSide ? leftIsShooting : rightIsShooting && robot.isInTriangle(follower.getPose())) {
            chute.start();
            robot.lock.open();

            if (shooter.launcherMotor.getCorrectedVelocity() - (isLeftSide ? lastVelocityLeft : lastVelocityRight) < -100) {
                robot.lock.close();
                chute.stop();
                if (isLeftSide) {
                    leftIsShooting = false;
                } else {
                    rightIsShooting = false;
                }
                gamepad1.rumble(500);
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
        if (error < 0) {
            power = 0;
        }
        double powerClamped = Range.clip(power, 0, 1);
        telemetry.addData("power", power);
        telemetry.addData("error", error);
        shooter.launcherMotor.set(powerClamped);

        //shooter.launcherMotor.set(targetPosition);
    }
}