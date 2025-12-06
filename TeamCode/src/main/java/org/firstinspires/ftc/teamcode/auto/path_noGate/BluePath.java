package org.firstinspires.ftc.teamcode.auto.path_noGate;

import static org.firstinspires.ftc.teamcode.auto.path_noGate.Poses.shootPosition;
import static org.firstinspires.ftc.teamcode.auto.path_noGate.Poses.startPosition;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
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
    public PathChain LThreeToCloseShoot;
    public PathChain StartToTag;
    public PathChain TagToShoot;


    public BluePath(Follower follower) {
        StartToShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPosition, shootPosition)
                )
                .setLinearHeadingInterpolation(startPosition.getHeading(), shootPosition.getHeading())
                .build();

        StartToTag = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPosition, new Pose(72, 72))
                )
                .setLinearHeadingInterpolation(startPosition.getHeading(), Math.toRadians(90))
                .build();

        TagToShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(72, 72), shootPosition)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), shootPosition.getHeading())
                .build();

        ShootToLOne = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPosition, new Pose(45.360, 83.520))
                )
                .setLinearHeadingInterpolation(shootPosition.getHeading(), Math.toRadians(180))
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
                        new BezierLine(new Pose(20.400, 83.520), shootPosition)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPosition.getHeading())
                .build();

        ShootToLTwo = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPosition, new Pose(44.640, 60.000))
                )
                .setLinearHeadingInterpolation(shootPosition.getHeading(), Math.toRadians(180))
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
                        new BezierLine(new Pose(19.200, 60.000), shootPosition)
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPosition.getHeading())
                .build();

        ShootToLThree = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPosition, new Pose(43.440, 35.500))
                )
                .setLinearHeadingInterpolation(shootPosition.getHeading(), Math.toRadians(180))
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

        LThreeToCloseShoot = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(18.720, 35.500),
                                new Pose(50.570, 46.488),
                                shootPosition
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), shootPosition.getHeading())
                .build();
    }
}