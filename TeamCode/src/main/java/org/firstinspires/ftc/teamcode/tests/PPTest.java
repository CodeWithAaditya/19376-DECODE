
package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.common.drive.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PPTest extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private ElapsedTime pathTimer, actionTimer, opmodeTimer;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        opmodeTimer.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(34, 137, Math.toRadians(180)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain line2;
        public PathChain line3;
        public PathChain line4;
        public PathChain line5;
        public PathChain line6;
        public PathChain line7;
        public PathChain line8;
        public PathChain line9;
        public PathChain line10;
        public PathChain line11;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(34.000, 137.000),
                                    new Pose(53.768, 97.942),
                                    new Pose(57.532, 86.826)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            line2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.532, 86.826),
                                    new Pose(61.787, 70.884),
                                    new Pose(55.748, 57.387),
                                    new Pose(13.287, 59.768)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            line3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.287, 59.768),
                                    new Pose(55.994, 55.671),
                                    new Pose(59.342, 77.574)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155))

                    .build();

            line4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.342, 77.574),
                                    new Pose(54.497, 52.555),
                                    new Pose(13.006, 62.245)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(155))

                    .build();

            line5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.006, 62.245),
                                    new Pose(42.626, 57.652),
                                    new Pose(59.342, 77.690)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(155))

                    .build();

            line6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.342, 77.690),
                                    new Pose(42.516, 57.555),
                                    new Pose(13.239, 62.245)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(155))

                    .build();

            line7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.239, 62.245),
                                    new Pose(42.742, 57.658),
                                    new Pose(59.342, 77.806)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(155))

                    .build();

            line8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.342, 77.806),
                                    new Pose(49.148, 85.019),
                                    new Pose(14.865, 84.194)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            line9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(14.865, 84.194),
                                    new Pose(45.103, 85.806),
                                    new Pose(61.781, 80.129)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            line10 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(61.781, 80.129),
                                    new Pose(67.958, 56.477),
                                    new Pose(59.920, 32.870),
                                    new Pose(41.758, 35.445),
                                    new Pose(12.774, 35.768)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(180))

                    .build();

            line11 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(12.774, 35.768),
                                    new Pose(62.245, 60.735),
                                    new Pose(65.845, 91.394)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line2);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line3);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line4);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line5);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line6);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line7);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line8);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line9);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line10);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line11);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    setPathState(-1); // Stop
                }
                break;
        }
    }

    public void setPathState(int pState) { pathState = pState; pathTimer.reset(); }
}
    