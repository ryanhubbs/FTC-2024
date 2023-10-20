package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class AutoAlign {
    public Align(IMU imu, double SPEED_MULTIPLIER, DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double range = Constants.AUTO_ALIGN_RANGE;
        double speed = Constants.ALIGN_SPEED * SPEED_MULTIPLIER;

        while (!((360 - range) <= Abs(botHeading)) ||   // if robot rotated left within range
            !(Abs(botHeading) <= range))                // if robot rotated right within range
        {
            frontLeftMotor.setPower(speed);
            backLeftMotor.setPower(speed);
            frontRightMotor.setPower(-speed);
            backRightMotor.setPower(-speed);
        }
    }
}