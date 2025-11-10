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
    GamepadEx driverGamepad;

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

        Commands commandsList = new Commands();
        driverGamepad = new GamepadEx(gamepad1);

        Button shootRightButton = new GamepadButton(
                driverGamepad, GamepadKeys.Button.RIGHT_BUMPER
        ).whenPressed(
                new RepeatCommand(
                        (commandsList.new Shoot(robot.shooterRight, robot.chuteRight, robot.lock, 380)),
                        2
                )
        );

        class GamepadLeftTrigger extends Trigger {
            @Override
            public boolean get() {
                return gamepad1.left_trigger > 0;
            }
        }

        Trigger hold = new GamepadLeftTrigger().whenActive(
                commandsList.new SetShooterVelocity(robot.shooterLeft, 380)
        );

        Button intakeButton = new GamepadButton(
                driverGamepad, GamepadKeys.Button.A
        ).toggleWhenPressed(commandsList.new RunIntake(robot.chuteRight, robot.chuteLeft, robot.intakeFront, robot.intakeBack));
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

        if (gamepad1.yWasPressed()) {
            follower.setPose(follower.getPose().withHeading(0));
        }

        CommandScheduler.getInstance().run();







        telemetryM.debug("position", follower.getPose());
        telemetryM.update(telemetry);
    }

    @Override
    public void stop() {
        robot.endPositionBlackboard.set(follower.getPose());
    }
}