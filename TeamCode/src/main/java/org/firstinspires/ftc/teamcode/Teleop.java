package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Telewop", group="")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor armPivot = hardwareMap.dcMotor.get("armPivot");

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");


        armPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);


        Gamepad driverController = gamepad1;
        Gamepad operatorController = gamepad2;


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double o_ly = operatorController.left_stick_y;

            double d_ly = -driverController.left_stick_y;
            double d_lx = driverController.left_stick_x;
            double d_rx = driverController.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double speedMultiplier = 1;
            double rotX = d_lx * Math.cos(-botHeading) - d_ly * Math.sin(-botHeading);
            double rotY = d_lx * Math.sin(-botHeading) + d_ly * Math.cos(-botHeading);
            rotX = rotX * 1.1;

            // Driver
            if (driverController.right_bumper) {
                speedMultiplier = 8;
            }

            if (driverController.start) {
                imu.resetYaw();
            }



            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(d_rx), 1);
            double frontLeftPower = (rotY + rotX + d_rx) / denominator;
            double backLeftPower = (rotY - rotX + d_rx) / denominator;
            double frontRightPower = (rotY - rotX - d_rx) / denominator;
            double backRightPower = (rotY + rotX - d_rx) / denominator;


            frontLeft.setPower(frontLeftPower / speedMultiplier);
            backLeft.setPower(backLeftPower / speedMultiplier);
            frontRight.setPower(frontRightPower / speedMultiplier);
            backRight.setPower(backRightPower / speedMultiplier);

            // Operator
            if (o_ly < -0.1 || o_ly > 0.1) {
                armPivot.setPower(Math.pow(o_ly,3.0));
            } else {
                armPivot.setPower(0);
            }
        }
    }
}
