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
    int lockTicks = 100;
    PIDFController headingPIDController = new PIDFController(new PIDFCoefficients(0, 0, 0, 0));
    static PIDFController launcherPidController = new PIDFController(new PIDFCoefficients(0.0002, 0, 0, 0));
    double headingError = 0;
    double headingOffset = 0;
    double velocityError = 0;
    double thePowerForTheLauncher = 0;
    double targetVelocityPid = 0;
    Hardware.Teams team = Hardware.Teams.BLUE;
    Hardware robot;
    GamepadEx driverGamepad;

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

        Button shootRightButton = new GamepadButton(
                driverGamepad, GamepadKeys.Button.RIGHT_BUMPER
        ).whenPressed(
                commandsList.new Shoot(robot.shooterLeft, robot.chuteLeft, robot.lock, 1000)
        );

        Button shootLeftButton = new GamepadButton(
                driverGamepad, GamepadKeys.Button.LEFT_BUMPER
        ).whenPressed(
                commandsList.new Shoot(robot.shooterRight, robot.chuteRight, robot.lock, 1000)
        );

        Button sortButton = new GamepadButton(
                driverGamepad, GamepadKeys.Button.B
        ).whenHeld(
                commandsList.new DetectColorAndSort(robot.intakeBack.colorSensor, robot.intakeBack.colorSensor, robot.sorter)
        );

        class GamepadLeftTrigger extends Trigger {
            @Override
            public boolean get() {
                return gamepad1.right_trigger > 0;
            }
        }

        Trigger hold = new GamepadLeftTrigger().whenActive(
                commandsList.new KeepShooterVelocity(robot.shooterRight, 1000)
        ).whenInactive(
                commandsList.new SetShooterVelocity(robot.shooterRight, 0)
        );

        Button lockButton = new GamepadButton(
                driverGamepad, GamepadKeys.Button.DPAD_UP
        ).toggleWhenPressed(new ParallelCommandGroup(commandsList.new OpenLock(robot.lock), commandsList.new SpinChute(robot.chuteRight, false)), commandsList.new CloseLock(robot.lock));

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
                        false
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

        if (gamepad1.left_trigger > 0) {
            shoot(targetVelocity);

        } else {
            robot.shooterRight.launcherMotor.set(0);
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

        if (gamepad1.xWasPressed()) {
            headingLock = !headingLock;
        }

        if (gamepad1.dpad_left) {
            pitchAngle++;
        } else if (gamepad1.dpad_right) {
            pitchAngle--;
        }
        robot.shooterRight.setPitchAngle(pitchAngle, true);
        robot.shooterLeft.setPitchAngle(pitchAngle, false);

        CommandScheduler.getInstance().run();







        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetryM.addData("launcherPower", robot.shooterRight.launcherMotor.get());
        telemetryM.addData("launcherPower", robot.shooterRight.launcherMotor.getCorrectedVelocity());
        telemetryM.addData("team", team);
        telemetryM.addData("distance to team goal", Hardware.distanceToGoal(team, follower.getPose()));
        telemetryM.addData("launcher target power", thePowerForTheLauncher);
        telemetryM.addData("launcher error", velocityError);

        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        robot.endPositionBlackboard.set(follower.getPose());
    }

    void shoot(double targetPosition) {
        if (robot.shooterRight.launcherMotor.getCorrectedVelocity() > targetPosition) {
            robot.shooterRight.launcherMotor.set(0.001);
            atVelTicks++;
            if (atVelTicks > 5) {
                robot.chuteRight.start();
                robot.lock.open();
            }
        } else {
            robot.shooterRight.launcherMotor.set(1);
            atVelTicks = 0;
        }
    }
}