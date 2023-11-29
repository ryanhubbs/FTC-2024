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
        // defines hardware
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        DcMotor armMotor = hardwareMap.dcMotor.get("intakeOne");
        Servo clawServo = hardwareMap.servo.get("clawServo");
        Servo wristServo = hardwareMap.servo.get("wristServo");


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu.initialize(parameters);

        String CURRENT_PRESET = "STOW";
        String CLAW_STATE = "CLOSED";
        if (Constants.MOVE_ON_INIT) {
            clawServo.setPosition(Constants.CLAW_CLOSED);
            wristServo.setPosition(Constants.PRESET_STOW);
        }

        // init finishes here
        waitForStart(); // wait for start button on driver hub

        if (isStopRequested()) return;

        // teleop mode starts
        //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive()) {
            // driver sticks
            double d_y = gamepad1.left_stick_y;
            double d_x = gamepad1.left_stick_x * 1.1;
            double d_rx = gamepad1.right_stick_x;

            // operator sticks
            double o_ly = gamepad2.left_stick_y;
            double o_ry = gamepad2.right_stick_y;


            // DRIVER KEYS
            double denominator = Math.max(Math.abs(d_y) + Math.abs(d_x) + Math.abs(d_rx), 1);
            double frontLeftPower = (d_y + d_x + d_rx) / denominator;
            double backLeftPower = (d_y - d_x + d_rx) / denominator;
            double frontRightPower = (d_y - d_x - d_rx) / denominator;
            double backRightPower = (d_y + d_x - d_rx) / denominator;
            double DRIVETRAIN_MULTIPLIER = 1;

            if (gamepad1.left_bumper) { // if the left bumper is held then divide motor speeds by Constants.DRIVETRAIN_MULTIPLIER
                DRIVETRAIN_MULTIPLIER = Constants.SLOW_MULTIPLIER;
            }

            // assign each motor their speeds
            frontLeftMotor.setPower(frontLeftPower / DRIVETRAIN_MULTIPLIER);
            backLeftMotor.setPower(backLeftPower / DRIVETRAIN_MULTIPLIER);
            frontRightMotor.setPower(frontRightPower / DRIVETRAIN_MULTIPLIER);
            backRightMotor.setPower(backRightPower / DRIVETRAIN_MULTIPLIER);


////////////////////////////////////////////////////////////////////////////////////////////////////

            // OPERATOR KEYS
            // armMotor
//            if (gamepad2.y) {
//                armMotor.setPower(-0.25);
//            } else if (gamepad2.x) {
//                armMotor.setPower(0.25);
//            } else {
//                armMotor.setPower(0);
//            }

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
//            } else {
//                armMotor.setPower(0);
//            }

            telemetry.addData("Arm Speed", o_ry / 1.5);

            // claw open / closing
            if (gamepad2.a) {
                clawServo.setPosition(Constants.CLAW_OPEN);
                CLAW_STATE = "OPEN";
            } else if (gamepad2.b) {
                clawServo.setPosition(Constants.CLAW_CLOSED);
                CLAW_STATE = "CLOSED";
            }

            // claw wrist presets
            if (gamepad2.dpad_down) {
                wristServo.setPosition(Constants.PRESET_INTAKE);
                CURRENT_PRESET = "INTAKE";
            } else if (gamepad2.dpad_left) {
                wristServo.setPosition(Constants.PRESET_STOW);
                CURRENT_PRESET = "STOW";
            } else if (gamepad2.dpad_right) {
                wristServo.setPosition((Constants.PRESET_SCORE));
                CURRENT_PRESET = "SCORE";
            }

            if (gamepad2.x) {
                armMotor.setTargetPosition(180);
                //armMotor.setPower(0.5);
//                if (Math.abs(armMotor.getTargetPosition() - armMotor.getCurrentPosition()) < 4) {
//                };
            }

            telemetry.addData("Wrist Position", wristServo.getPosition());
            telemetry.addData("Intake Preset", CURRENT_PRESET);
            telemetry.addData("Claw State", CLAW_STATE);
            telemetry.update();
        }

    }
}