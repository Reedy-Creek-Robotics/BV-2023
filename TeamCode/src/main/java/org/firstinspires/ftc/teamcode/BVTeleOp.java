package org.firstinspires.ftc.teamcode;
import android.media.Image;
import android.transition.Slide;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;

@TeleOp
public class BVTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime timeSinceManualMode = new ElapsedTime();
        DcMotor Slide = hardwareMap.get(DcMotor.class, "Slide");
        DcMotor SupensionSlide = hardwareMap.get(DcMotor.class, "SuspensionSlide");
        Servo PlaneLaunchServo = hardwareMap.get(Servo.class, "LaunchServo");
        CRServo RollerIntake = hardwareMap.get(CRServo.class, "RollerIntake");
        Servo ClawRotation = hardwareMap.get(Servo.class, "ClawRotation");
        Servo Claw = hardwareMap.get(Servo.class, "Claw");

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BackRight");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();


        //--------------------------------------------

        int CurrentPosition;
        int SlideStop = 0;
        double ClawPos;
        double OpenClaw = 0.5;
        double ClosedClaw = 0.33;
        double RollerPow = 0.1;
        double CurrentPower = 0.8;
        int currentSlideTick = 0;
        boolean manualSlide = false;
        double slidePowerStop = 0.05;
        double slideElapsedTime = 0;

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        //Switch direction if servo runs backwards
        PlaneLaunchServo.setDirection(Servo.Direction.FORWARD);
        PlaneLaunchServo.setPosition(PlaneLaunchServo.getPosition());
        Slide.setDirection(DcMotor.Direction.REVERSE);
        Slide.setTargetPosition(0);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            //Temporarily flipped the x and y variables
            double correctionFactor = 1.1;
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * correctionFactor; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            int SlideCurrentPos = Slide.getCurrentPosition();
            //double slideY = gamepad2.right_stick_y;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]


            float botHeading = -imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontleft = (rotY + rotX + rx) / denominator;
            double backleft = (rotY - rotX + rx) / denominator;
            double frontright = (rotY - rotX - rx) / denominator;
            double backright = (rotY + rotX - rx) / denominator;

            //slide transfer
            double ytransfer = gamepad2.right_stick_y;
            Slide.setPower(ytransfer);

            //suspension slide

            double travel = gamepad2.left_stick_y;
            SupensionSlide.setPower(travel);

            //claw
            if (gamepad2.left_bumper && runtime.milliseconds() > 1000) {
                Claw.setPosition(ClosedClaw);
            }
            if (gamepad2.right_bumper && runtime.milliseconds() > 1000) {
                Claw.setPosition(OpenClaw);
            }

            //plane launcher
            if (gamepad1.left_bumper) {
                PlaneLaunchServo.setPosition(0.0);
            }
            if (gamepad1.right_bumper) {
                PlaneLaunchServo.setPosition(1.0);
            }
            //roller intake
            if (gamepad1.a) {
                RollerIntake.setPower(0.5);
            }
            if (gamepad1.b) {
                RollerIntake.setPower(-0.5);
            }
            if (gamepad1.x) {
                RollerIntake.setPower(0);
            }



            /*

            if (gamepad1.dpad_up) {
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setTargetPosition(SlideCurrentPos += 1000);
                Slide.setPower(1);
            }
            if (gamepad1.dpad_down) {
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setTargetPosition(SlideCurrentPos -= 1000);
                Slide.setPower(1);
            }
            if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
                Slide.setPower(0);
            }
            if (Slide.getCurrentPosition() < 0) {
                Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                SlideCurrentPos = 0;
            }
            if (gamepad1.right_bumper && slideElapsedTime >= 0.25) {
                slidePowerStop += 0.05;
                slideElapsedTime = 0;
            }
            if (gamepad1.left_bumper && slideElapsedTime >= 0.25) {
                slidePowerStop -= 0.05;
                slideElapsedTime = 0;
            }
            if (gamepad1.right_trigger >= 0.6) {
                Slide.setTargetPosition(SlideCurrentPos -= 1000);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(slidePowerStop);
            }
            if (slidePowerStop < 0) {
                slidePowerStop = 0;
            }
            if (slidePowerStop > 1) {
                slidePowerStop = 1;
            }

            //slides

            /*if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            }*/

            /*ClawPos = ClawServo.getPosition();
            SlideStop = Slide1.getTargetPosition();
            CurrentPosition = Slide1.getCurrentPosition();*/

            slideElapsedTime += 0.01;

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Current Position:", Slide.getCurrentPosition());
            telemetry.addData("Target Position:", Slide.getTargetPosition());
            telemetry.addData("Current Power:", CurrentPower);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("PlaneServo Pos:", PlaneLaunchServo.getPosition());
            telemetry.addData("Current Idle Slide Power:", slidePowerStop);
            telemetry.addData("Slide Elapsed Time Var:", slideElapsedTime);

            motorFrontLeft.setPower(frontleft);
            motorBackLeft.setPower(backleft);
            motorFrontRight.setPower(frontright);
            motorBackRight.setPower(backright);

            telemetry.update();
        }
    }
}