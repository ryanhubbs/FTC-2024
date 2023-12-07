package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name = "TeleWopyy")
public class MecanumTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Defines the drivetrain motors.
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        // Defines the arm/wrist servos and motor.
        DcMotor armMotor = hardwareMap.dcMotor.get("intakeOne");
        Servo clawServo = hardwareMap.servo.get("clawServo");
        Servo wristServo = hardwareMap.servo.get("wristServo");
        Servo forarmServo = hardwareMap.servo.get("middleArmServo");

        // Flip right front and right back motors directions.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motors to run using encoder.
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set arm motor to stop and reset run mode.
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the imu.
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Create new imu params using RevHubBasedOrientation.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        // Initialize imu params.
        imu.initialize(parameters);

        // Set the current
        String currentForearmPreset = "STOW";
        String currentWristPreset = "STOW";
        String CLAW_STATE = "CLOSED";


        if (Constants.MOVE_ON_INIT) {
            forarmServo.setPosition(Constants.PRESET_F_STOW_POS);
            wristServo.setPosition(Constants.PRESET_W_STOW_POS);
            clawServo.setPosition(Constants.CLAW_CLOSED);
        }

        // Initialization finishes here

        // wait for start button on driver hub
        waitForStart();

        if (isStopRequested()) return;

        // teleop mode starts
        //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive()) {
            // driver sticks
            double d_y = gamepad1.left_stick_y;
            double d_x = gamepad1.left_stick_x * 1.1;
            double d_rx = gamepad1.right_stick_x;

            // Operator sticks
            double o_ly = gamepad2.left_stick_y;
            double o_ry = gamepad2.right_stick_y;


            // DRIVER KEYS
            double denominator = Math.max(Math.abs(d_y) + Math.abs(d_x) + Math.abs(d_rx), 1);
            double frontLeftPower = (d_y + d_x + d_rx) / denominator;
            double backLeftPower = (d_y - d_x + d_rx) / denominator;
            double frontRightPower = (d_y - d_x - d_rx) / denominator;
            double backRightPower = (d_y + d_x - d_rx) / denominator;
            double DRIVETRAIN_MULTIPLIER = 1;

            // if the left bumper is held then divide motor speeds by Constants.DRIVETRAIN_MULTIPLIER
            if (gamepad1.left_bumper) {
                DRIVETRAIN_MULTIPLIER = Constants.SLOW_MULTIPLIER;
            }

            // Assign each motor their speeds.
            frontLeftMotor.setPower(frontLeftPower / DRIVETRAIN_MULTIPLIER);
            backLeftMotor.setPower(backLeftPower / DRIVETRAIN_MULTIPLIER);
            frontRightMotor.setPower(frontRightPower / DRIVETRAIN_MULTIPLIER);
            backRightMotor.setPower(backRightPower / DRIVETRAIN_MULTIPLIER);

            // Check for buttons down and move the humerous by a multiplier times the direction.
            if (gamepad2.left_trigger > 0) {
                armMotor.setPower(0.5 * gamepad2.left_trigger);
            } else {
                armMotor.setPower(0);
            }

            if (gamepad2.right_trigger > 0) {
                armMotor.setPower(-0.75 * gamepad2.right_trigger);
            } else {
                armMotor.setPower(0);
            }

            telemetry.addData("Arm Speed", o_ry / 1.5);

            // claw open / closing
            if (gamepad2.a) {
                clawServo.setPosition(Constants.CLAW_OPEN);
                CLAW_STATE = "OPEN";
            } else if (gamepad2.b) {
                clawServo.setPosition(Constants.CLAW_CLOSED);
                CLAW_STATE = "CLOSED";
            }

            // Check for buttons down and then trigger certain presets.
            if (gamepad2.dpad_down) {
                wristServo.setPosition(Constants.PRESET_W_INTAKE_POS);
                forarmServo.setPosition(Constants.PRESET_F_INTAKE_POS);
                currentForearmPreset = "INTAKE";
                currentWristPreset = "INTAKE";
            }

            else if (gamepad2.dpad_left) {
                wristServo.setPosition(Constants.PRESET_W_STOW_POS);
                forarmServo.setPosition(Constants.PRESET_F_STOW_POS);
                currentForearmPreset = "STOW";
                currentWristPreset = "STOW";
            }

            else if (gamepad2.dpad_right) {
                forarmServo.setPosition(Constants.PRESET_F_SCORE);
                wristServo.setPosition(Constants.PRESET_W_SCORE);
                currentForearmPreset = "SCORE";
                currentWristPreset = "SCORE";
            }

            if (gamepad2.x) {
                armMotor.setTargetPosition(180);
            }

            telemetry.addData("Wrist Position", wristServo.getPosition());
            telemetry.addData("Wrist Preset", currentWristPreset);
            telemetry.addData("Forearm Position", forarmServo.getPosition());
            telemetry.addData("Forearm Preset", currentForearmPreset);
            telemetry.addData("Claw State", CLAW_STATE);
            telemetry.update();
        }

    }
}