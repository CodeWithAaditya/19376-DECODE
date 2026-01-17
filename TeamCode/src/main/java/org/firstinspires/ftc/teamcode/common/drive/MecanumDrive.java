// Combined and extended Mecanum drive class with:
// - GoBILDA Pinpoint Odometry integration
// - Manual/Auto modes
// - Linear interpolated point-to-point movement with heading

package org.firstinspires.ftc.teamcode.common.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MecanumDrive {
    private DcMotor driveFL, driveFR, driveBL, driveBR;
    public GoBildaPinpointDriver odo;

    public MecanumDrive(HardwareMap hardwareMap) {
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");

        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveFR.setDirection(DcMotor.Direction.FORWARD);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveBR.setDirection(DcMotor.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(8, -165, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.resetPosAndIMU();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        driveFL.setZeroPowerBehavior(behavior);
        driveFR.setZeroPowerBehavior(behavior);
        driveBL.setZeroPowerBehavior(behavior);
        driveBR.setZeroPowerBehavior(behavior);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        driveFL.setPower(fl);
        driveFR.setPower(fr);
        driveBL.setPower(bl);
        driveBR.setPower(br);
    }

    public void drive(double axial, double lateral, double yaw) {
            double powerFL = axial + lateral + yaw;
            double powerFR = axial - lateral - yaw;
            double powerBL = axial - lateral + yaw;
            double powerBR = axial + lateral - yaw;

            double max = Math.max(Math.abs(powerFL), Math.abs(powerFR));
            max = Math.max(max, Math.abs(powerBL));
            max = Math.max(max, Math.abs(powerBR));

            if (max > 1.0) {
                powerFL /= max;
                powerFR /= max;
                powerBL /= max;
                powerBR /= max;
            }

            setMotorPowers(powerFL, powerFR, powerBL, powerBR);
    }

    public void driveFieldCentric(double axial, double lateral, double yaw) {
        odo.update();  // Update heading from odometry

        double heading = odo.getPosition().getHeading(AngleUnit.RADIANS);

        // Rotate joystick input vector by -heading (field-centric)
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double fieldAxial = axial * cos - lateral * sin;
        double fieldLateral = axial * sin + lateral * cos;

        // Calculate motor powers
        double powerFL = fieldAxial + fieldLateral + yaw;
        double powerFR = fieldAxial - fieldLateral - yaw;
        double powerBL = fieldAxial - fieldLateral + yaw;
        double powerBR = fieldAxial + fieldLateral - yaw;

        // Normalize powers
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(powerFL), Math.abs(powerFR)),
                Math.max(Math.abs(powerBL), Math.abs(powerBR))
        ));

        setMotorPowers(powerFL / max, powerFR / max, powerBL / max, powerBR / max);
    }

    public Pose2D getPose() {
        return odo.getPosition();
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
