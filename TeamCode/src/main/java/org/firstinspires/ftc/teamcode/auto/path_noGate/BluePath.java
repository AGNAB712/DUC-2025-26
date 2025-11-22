package org.firstinspires.ftc.teamcode.auto.path_noGate;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class BluePath {

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

    public BluePath(Follower follower) {
        StartToShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(29.280, 130.800), new Pose(56.880, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        ShootToLOne = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.880, 84.000), new Pose(45.360, 83.520))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        LOne = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45.360, 83.520), new Pose(20.400, 83.520))
                )
                .setTangentHeadingInterpolation()
                .build();

        LOneToShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.400, 83.520), new Pose(56.400, 83.760))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        ShootToLTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.400, 83.760), new Pose(44.640, 60.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        LTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.640, 60.000), new Pose(19.200, 60.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        LTwoToShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.200, 60.000), new Pose(56.160, 83.760))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        ShootToLThree = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.160, 83.760), new Pose(43.440, 35.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .build();

        LThree = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(43.440, 35.500), new Pose(18.720, 35.500))
                )
                .setTangentHeadingInterpolation()
                .build();

        LThreeToFarShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.720, 35.500), new Pose(68.880, 17.280))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(125))
                .build();
    }
}