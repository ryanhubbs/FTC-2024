package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name = "TeleWopy")
public class MecanumTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // define motors from driver hub config
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        DcMotor intakeOne = hardwareMap.dcMotor.get("intakeOne");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);


        // imu built into control hub
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu.initialize(parameters);

        // auto align constants
        double AUTO_ALIGN_RANGE = Constants.AUTO_ALIGN_RANGE;
        double AUTO_ALIGN_SPEED = Constants.AUTO_ALIGN_SPEED * Constants.AUTO_ALIGN_SPEED_MULTIPLIER;
        waitForStart();

        if (isStopRequested()) return;

        // teleop mode starts
        while (opModeIsActive()) {
            // driver stuff
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // operator stuff
            double o_ly = gamepad2.left_stick_y;


            // zero imu yaw axis
            if (gamepad1.start) {
                imu.resetYaw();
            }

            // variables for robot imu heading
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);


            // math for field centric drive
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            double slowSpeed = 5;

            // display encoder values for drivetrain
            telemetry.addData("Front Left Encoder", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right Encoder", frontRightMotor.getCurrentPosition());
            telemetry.addData("Back Left Encoder", backLeftMotor.getCurrentPosition());
            telemetry.addData("Back Right Encoder", backRightMotor.getCurrentPosition());

            // slow multiplier button
            if (gamepad1.left_bumper) {
                frontLeftMotor.setPower(frontLeftPower / slowSpeed);
                backLeftMotor.setPower(backLeftPower / slowSpeed);
                frontRightMotor.setPower(frontRightPower / slowSpeed);
                backRightMotor.setPower(backRightPower / slowSpeed);
            } else { // button not pressed so dont slow
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }

            if (gamepad1.a) { // loops until robot is within AUTO_ALIGN_RANGE of the IMU zero 
                while (!((360 - AUTO_ALIGN_RANGE) <= Math.abs(botHeadingDeg)) ||     // if robot rotated left within range
                    !(Math.abs(botHeadingDeg) <= AUTO_ALIGN_RANGE))                  // if robot rotated right within range
                { // sets motor power based off constants
                    frontLeftMotor.setPower(AUTO_ALIGN_SPEED);
                    backLeftMotor.setPower(AUTO_ALIGN_SPEED);
                    frontRightMotor.setPower(-AUTO_ALIGN_SPEED);
                    backRightMotor.setPower(-AUTO_ALIGN_SPEED);

                    telemetry.addData("Aligning", true);
                    telemetry.update();
                }
                // robot reached goal
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);

                telemetry.addData("Aligning", false);
            }

            // current position of joint 1 on the intake arm
            double intakeOnePos = intakeOne.getCurrentPosition();

            if ( // if operator left stick is within deadzone
                (o_ly > 0.1 || o_ly < -0.1)
            ) {
                if (o_ly < 0 && intakeOnePos < Constants.ONE_MAX) {
                    intakeOne.setPower(o_ly / Constants.ONE_SPEED_MULTIPLIER);
                } else if (o_ly > 0 && intakeOnePos > Constants.ONE_MIN) {
                    intakeOne.setPower(o_ly / Constants.ONE_SPEED_MULTIPLIER);
                }
                
            } else {
                intakeOne.setPower(0);
            }

            telemetry.addData("Arm One Speed", o_ly);
            telemetry.addData("Arm One Angle", intakeOne.getCurrentPosition());
            telemetry.update();
        }

    }
}