package org.firstinspires.ftc.teamcode.common.drive;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.pedropathing.control.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.5)
            .forwardZeroPowerAcceleration(-35.80215300875694)
            .lateralZeroPowerAcceleration(-74.1383316133329)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.22, 0, 0.02, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.06, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0003, 0.6, 0.06))
            .centripetalScaling(0.00045);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("driveFR")
            .rightRearMotorName("driveBR")
            .leftRearMotorName("driveBL")
            .leftFrontMotorName("driveFL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(83.09)
            .yVelocity(66.22);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-40)
            .strafePodX(-162)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
