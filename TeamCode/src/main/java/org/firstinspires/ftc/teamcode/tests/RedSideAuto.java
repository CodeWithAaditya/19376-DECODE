package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Intake;
import org.firstinspires.ftc.teamcode.common.Shooter;
import org.firstinspires.ftc.teamcode.common.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.common.drive.DriveToPoint;

import java.util.Locale;

@Autonomous(name="Red Side Auto", group="Auto")

public class RedSideAuto extends LinearOpMode {

    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    ElapsedTime timer = null;

    Intake intake;
    Shooter shooter;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        SHOOT_PRELOAD,
        SHOOT_2ND,
        DRIVE_TO_TARGET_2,
        WAIT_TO_CLAW,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5,
    }

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,600, 900,AngleUnit.DEGREES,-145);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, 700, 1350, AngleUnit.DEGREES, -180);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM,-200,1350, AngleUnit.DEGREES,-180);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM,600, 900,AngleUnit.DEGREES,-145);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM,-200,1350, AngleUnit.DEGREES,-180);

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "driveFL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "driveFR");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "driveBL");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "driveBR");

        timer = new ElapsedTime();

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(8, -165); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();
            shooter.update();

            switch (stateMachine){
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    shooter.setShooterVelocity(1650);
                    shooter.setHoodServoPos(0.2);
                    break;
                case DRIVE_TO_TARGET_1:
                    if (nav.driveTo(odo.getPosition(), TARGET_1, 0.7, 0)){
                        telemetry.addLine("at position #1!");
                        intake.setIntakeState(Intake.IntakeState.ON);
                        timer.reset();
                        stateMachine = StateMachine.SHOOT_PRELOAD;
                    }
                    break;
                case SHOOT_PRELOAD:
                    if(timer.time()>5){
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    if (nav.driveTo(odo.getPosition(), TARGET_2, 0.7, 0)){
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if(intake.isBallInGrabdexer()){
                        intake.setGrabdexerState(Intake.GrabdexerState.MID);
                    }
                    if(nav.driveTo(odo.getPosition(), TARGET_3, 0.7, 0)){
                        telemetry.addLine("at position #3");
                        intake.setIntakeState(Intake.IntakeState.OFF);
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if(nav.driveTo(odo.getPosition(),TARGET_4,0.7,0)){
                        telemetry.addLine("at position #4");
                        intake.setGrabdexerState(Intake.GrabdexerState.IN);
                        intake.setIntakeState(Intake.IntakeState.ON);
                        timer.reset();
                        stateMachine = StateMachine.SHOOT_2ND;
                    }
                    break;
                case SHOOT_2ND:
                    if(timer.time()>5){
                        intake.setIntakeState(Intake.IntakeState.OFF);
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if(nav.driveTo(odo.getPosition(),TARGET_5,0.7,0)){
                        shooter.setShooterVelocity(0);
                        telemetry.addLine("There!");
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;
            }


            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("current state:",stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();

        }
    }}
