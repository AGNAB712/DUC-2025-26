package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
    Hardware robot = new Hardware();
    @IgnoreConfigurable
    Follower follower;

    GamepadEx gamepadDrive;
    GamepadEx gamepadSubsystem;

    static boolean traveling = false;
    private Supplier<PathChain> pathChain;

    @Override
    public void init() {
        gamepadDrive = new GamepadEx(gamepad1);
        gamepadSubsystem = new GamepadEx(gamepad2);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        robot.init(hardwareMap);

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(0, 0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

    }

    @Override
    public void init_loop() {
        telemetryM.debug("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryM.update(telemetry);
        follower.update();
        //drawCurrent();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        if (!traveling) {
            follower.setTeleOpDrive(gamepadDrive.getLeftY(), -gamepadDrive.getLeftX(), -gamepadDrive.getRightX(), true);
        }
        follower.update();

        Hardware.ArtifactType detectedArtifact = robot.csensor1.detectColor();
        robot.csensor1.trackColor(detectedArtifact);
        robot.sorter.updateServo(detectedArtifact);

        if (gamepadDrive.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            if (robot.getCurrentTeam() == Hardware.Teams.RED) {
                robot.setTeam(Hardware.Teams.BLUE);
                robot.shooter.yawServo.runToEncoderPosition(360);
                robot.sorter.green();
            } else {
                robot.setTeam(Hardware.Teams.RED);
                robot.shooter.yawServo.runToEncoderPosition(720);
                robot.sorter.purple();
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

        gamepadSubsystem.readButtons();
        gamepadDrive.readButtons();

        telemetryM.debug("team:" + robot.getCurrentTeam());
        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.debug("Color:" + detectedArtifact);
        telemetryM.debug("current sequence:" + robot.getCurrentArtifacts());
        telemetryM.debug("velocity x:" + follower.getVelocity().getXComponent());
        telemetryM.debug("velocity y:" + follower.getVelocity().getYComponent());
        robot.shooter.yawServo.update();
        telemetryM.debug("total angle:" + robot.shooter.yawServo.showTelemetryData()[0]);
        telemetryM.debug("rots:" + robot.shooter.yawServo.showTelemetryData()[1]);
        telemetryM.debug("distance:" + robot.shooter.yawServo.showTelemetryData()[2]);
        telemetryM.debug("direction:" + robot.shooter.yawServo.showTelemetryData()[3]);
        telemetryM.update(telemetry);



        //drawCurrentAndHistory();
    }
}