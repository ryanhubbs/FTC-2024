package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name="TeleWopy")
public class MecanumTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        DcMotor intakeOne = hardwareMap.dcMotor.get("intakeOne");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu.initialize(parameters);

        double AUTO_ALIGN_RANGE = Constants.AUTO_ALIGN_RANGE;
        double AUTO_ALIGN_SPEED = Constants.AUTO_ALIGN_SPEED * Constants.AUTO_ALIGN_SPEED_MULTIPLIER;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double o_ly = gamepad2.left_stick_y;

            if (gamepad1.start) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            double slowSpeed = 5;

            telemetry.addData("Heading", botHeading);

            if (gamepad1.left_bumper) {
                frontLeftMotor.setPower(frontLeftPower / slowSpeed);
                backLeftMotor.setPower(backLeftPower / slowSpeed);
                frontRightMotor.setPower(frontRightPower / slowSpeed);
                backRightMotor.setPower(backRightPower / slowSpeed);

            } else {
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }

            if (gamepad1.a) {
                while (!((360 - AUTO_ALIGN_RANGE) <= Abs(botHeadingDeg)) ||   // if robot rotated left within range
                    !(Abs(botHeadingDeg) <= AUTO_ALIGN_RANGE))                // if robot rotated right within range
                {
                    frontLeftMotor.setPower(AUTO_ALIGN_SPEED);
                    backLeftMotor.setPower(AUTO_ALIGN_SPEED);
                    frontRightMotor.setPower(-AUTO_ALIGN_SPEED);
                    backRightMotor.setPower(-AUTO_ALIGN_SPEED);

                    telemetry.addData("Aligning", true);
                    telemetry.update();
                }
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                
                telemetry.addData("Aligning", false);
            }

            double intakeOnePos = intakeOne.getCurrentPosition();

            if  ( 
                    (o_ly > 0.1 || o_ly < -0.1) && 
                    (intakeOnePos > Constants.ONE_MIN && intakeOnePos < Constants.ONE_MAX)
                )
            {
                intakeOne.setPower(o_ly / 4);
            } 
            else {
                intakeOne.setPower(0);
            }

            telemetry.addData("Arm One Speed", o_ly);
            telemetry.addData("Arm One Angle", intakeOne.getCurrentPosition());
            telemetry.update();
        }

    }
}