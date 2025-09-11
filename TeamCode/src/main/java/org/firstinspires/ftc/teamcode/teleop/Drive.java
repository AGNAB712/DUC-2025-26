package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.processors.ducProcessorArtifactsGreen;
import org.firstinspires.ftc.teamcode.processors.ducProcessorArtifactsPurple;
import org.firstinspires.ftc.vision.VisionPortal;

@Configurable
@TeleOp(name = "Teleop", group = "Teleop")
public class Drive extends OpMode {

    @IgnoreConfigurable
    static public TelemetryManager telemetryM;
    @IgnoreConfigurable
    Hardware robot = new Hardware();
    @IgnoreConfigurable
    Follower follower;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());
        robot.init(hardwareMap);
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
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        float[] hsv = robot.csensor1.getHSV();
        telemetryM.debug("R sensor 1:" + hsv[0]);
        telemetryM.debug("G sensor 1:" + hsv[1]);
        telemetryM.debug("B sensor 1:" + hsv[2]);
        Hardware.ArtifactType detectedArtifact = robot.csensor1.detectColor();
        telemetryM.debug("Color:" + detectedArtifact);
        telemetryM.debug("purple lcontour:" + robot.csensor1.contourAmount()[1]);
        telemetryM.debug("green lcontour:" + robot.csensor1.contourAmount()[0]);
        robot.csensor1.trackColor(detectedArtifact);
        telemetryM.debug("current sequence:" + robot.csensor1.getSequence());
        robot.sorter.updateServo(detectedArtifact);
        telemetryM.debug("velocity x:" + follower.getVelocity().getXComponent());
        telemetryM.debug("velocity y:" + follower.getVelocity().getYComponent());
        telemetryM.update(telemetry);



        //drawCurrentAndHistory();
    }
}