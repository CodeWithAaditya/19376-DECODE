package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.drive.Constants;

@Autonomous(name = "PathTest", group = "Test")
public class PathTest extends OpMode {

    private Follower follower;
    private int pathState;

    private PathChain path1;
    private PathChain line2;
    private PathChain line3;
    private PathChain line4;
    private PathChain line5;
    private PathChain path6;
    private PathChain line7;
    private PathChain line8;
    private PathChain line9;
    private PathChain line10;
    private PathChain line11;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(112.000, 135.000),
                                new Pose(93.000, 113.000),
                                new Pose(93.000, 94.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .build();

        line2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(93.000, 94.000),
                                new Pose(93.000, 62.000),
                                new Pose(127.000, 64.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        line3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(127.000, 64.500),
                                new Pose(118.000, 76.000),
                                new Pose(88.000, 95.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        line4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.000, 95.000),
                                new Pose(102.000, 77.000),
                                new Pose(103.000, 49.000),
                                new Pose(132.000, 63.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        line5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.000, 63.000),
                                new Pose(107.000, 83.000),
                                new Pose(88.000, 95.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.000, 95.000),
                                new Pose(102.000, 77.000),
                                new Pose(103.000, 49.000),
                                new Pose(132.000, 63.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        line7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.000, 63.000),
                                new Pose(107.000, 83.000),
                                new Pose(88.000, 95.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        line8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.000, 95.000),
                                new Pose(102.000, 77.000),
                                new Pose(103.000, 49.000),
                                new Pose(132.000, 63.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        line9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.000, 63.000),
                                new Pose(107.000, 83.000),
                                new Pose(88.000, 95.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        line10 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(88.000, 95.000),
                                new Pose(102.000, 77.000),
                                new Pose(103.000, 49.000),
                                new Pose(132.000, 63.000)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        line11 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(132.000, 63.000),
                                new Pose(110.000, 85.000),
                                new Pose(90.000, 99.000)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        pathState = 0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setStartingPose(new Pose(112.73, 136.14, Math.toRadians(-90)));
        pathState = 1;
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 1:
                follower.followPath(path1);
                pathState = 2;
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(line2);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(line3);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(line4);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(line5);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(path6);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(line7);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(line8);
                    pathState = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(line9);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(line10);
                    pathState = 11;
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(line11);
                    pathState = 12;
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    pathState = 13;
                }
                break;
            case 13:
                requestOpModeStop();
                break;
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
