package org.firstinspires.ftc.teamcode.auto.path_noGate;

import static org.firstinspires.ftc.teamcode.auto.path_noGate.Poses.shootPosition;
import static org.firstinspires.ftc.teamcode.auto.path_noGate.Poses.startPosition;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class RedPath {

    public PathChain StartToShoot;
    public PathChain ShootToLOne;
    public PathChain LOne;
    public PathChain LOneToShoot;
    public PathChain ShootToLTwo;
    public PathChain LTwo;
    public PathChain LTwoToShoot;
    public PathChain ShootToLThree;
    public PathChain LThree;
    public PathChain LThreeToFarShoot;
    public PathChain LThreeToCloseShoot;
    public PathChain StartToTag;
    public PathChain TagToShoot;


    public RedPath(Follower follower) {
        StartToTag = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPosition.mirror(), new Pose(72, 72))
                )
                .setLinearHeadingInterpolation(startPosition.mirror().getHeading(), Math.toRadians(90))
                .build();

        TagToShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(72, 72), shootPosition.mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), shootPosition.mirror().getHeading())
                .build();

        StartToShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPosition.mirror(), shootPosition.mirror())
                )
                .setLinearHeadingInterpolation(startPosition.mirror().getHeading(), shootPosition.mirror().getHeading())
                .build();

        ShootToLOne = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPosition.mirror(), new Pose(45.360, 83.520).mirror())
                )
                .setLinearHeadingInterpolation(shootPosition.mirror().getHeading(), 0)
                .build();

        LOne = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45.360, 83.520).mirror(), new Pose(20.400, 83.520).mirror())
                )
                .setTangentHeadingInterpolation()
                .build();

        LOneToShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.400, 83.520).mirror(), shootPosition.mirror())
                )
                .setLinearHeadingInterpolation(0, shootPosition.mirror().getHeading())
                .build();

        ShootToLTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPosition.mirror(), new Pose(44.640, 60.000).mirror())
                )
                .setLinearHeadingInterpolation(shootPosition.mirror().getHeading(), 0)
                .build();

        LTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.640, 60.000).mirror(), new Pose(19.200, 60.000).mirror())
                )
                .setTangentHeadingInterpolation()
                .build();

        LTwoToShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.200, 60.000).mirror(), shootPosition.mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), shootPosition.mirror().getHeading())
                .build();

        ShootToLThree = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPosition.mirror(), new Pose(43.440, 35.500).mirror())
                )
                .setLinearHeadingInterpolation(shootPosition.mirror().getHeading(), 0)
                .build();

        LThree = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(43.440, 35.500).mirror(), new Pose(18.720, 35.500).mirror())
                )
                .setTangentHeadingInterpolation()
                .build();

        LThreeToFarShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.720, 35.500).mirror(), new Pose(68.880, 17.280).mirror())
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(125))
                .build();

        LThreeToCloseShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(18.720, 35.500).mirror(),
                                new Pose(50.570, 46.488).mirror(),
                                shootPosition.mirror()
                        )
                )
                .setLinearHeadingInterpolation(0, shootPosition.mirror().getHeading())
                .build();
    }
}