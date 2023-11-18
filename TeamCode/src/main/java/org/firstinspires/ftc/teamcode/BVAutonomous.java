package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous
@Disabled
public class BVAutonomous extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    int backLeftTarget;
    int backRightTarget;
    int frontLeftTarget;
    int frontRightTarget;

    enum Direction {
        FORWARD,
        LEFT,
        BACKWARD,
        RIGHT
    }

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.dcMotor.get("FrontLeft");
        backLeft = hardwareMap.dcMotor.get("BackLeft");
        frontRight = hardwareMap.dcMotor.get("FrontRight");
        backRight = hardwareMap.dcMotor.get("BackRight");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        /*frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);*/

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {

            //Motor action methods below

            //Moves the robot forward at 0.6 power, 12 inches
            motorLinear(0.6, 12.0, Direction.FORWARD);

            while (backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy()) {
                //Repeat this loop between actions to have actions run one at a time
            }

            //Rotates the robot to the left at 0.6 power for 1 inch
            //motorRotate(0.6, 1.0, Direction.LEFT);

            telemetry.addLine("Path Complete");
            telemetry.update();

        }
    }

    public void motorLinear(double power, double inches, Direction direction) {


        double COUNTS_PER_MOTOR_REV = 2150.8;    // eg: TETRIX Motor Encoder
        double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
        double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        // Determine new target position, and pass to motor controller
        backLeftTarget = (int)(inches * COUNTS_PER_INCH);
        backRightTarget = (int)(inches * COUNTS_PER_INCH);
        frontLeftTarget = (int)(inches * COUNTS_PER_INCH);
        frontRightTarget = (int)(inches * COUNTS_PER_INCH);

        //Direction configuration (V2)
          if (direction == Direction.FORWARD) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        } if (direction == Direction.BACKWARD) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
              telemetry.addLine("Error: No valid direction");
              stop();
        }

        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);


        // Turn On RUN_TO_POSITION
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion
        backLeft.setPower(Math.abs(power));
        backRight.setPower(Math.abs(power));
        frontLeft.setPower(Math.abs(power));
        frontRight.setPower(Math.abs(power));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that ALL motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Running to", " %7d :%7d", backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget);
            telemetry.addData("Currently at", " at %7d :%7d", backLeft.getCurrentPosition(), backRight.getCurrentPosition(), frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn off RUN_TO_POSITION
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);

    }

    public void motorRotate(double power, double inches, Direction direction) {

        double COUNTS_PER_MOTOR_REV = 2150.8;    // eg: TETRIX Motor Encoder
        double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
        double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        // Determine new target position, and pass to motor controller
        backLeftTarget = (int)(inches * COUNTS_PER_INCH);
        backRightTarget = (int)(inches * COUNTS_PER_INCH);
        frontLeftTarget = (int)(inches * COUNTS_PER_INCH);
        frontRightTarget = (int)(inches * COUNTS_PER_INCH);

        //Direction configuration (V2)
        if (direction == Direction.LEFT) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        } if (direction == Direction.RIGHT) {
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            telemetry.addLine("Error: No valid direction");
            stop();
        }

        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);


        // Turn On RUN_TO_POSITION
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion
        backLeft.setPower(Math.abs(power));
        backRight.setPower(Math.abs(power));
        frontLeft.setPower(Math.abs(power));
        frontRight.setPower(Math.abs(power));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that ALL motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Running to", " %7d :%7d", backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget);
            telemetry.addData("Currently at", " at %7d :%7d", backLeft.getCurrentPosition(), backRight.getCurrentPosition(), frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn off RUN_TO_POSITION
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);

    }

    public void motorStrafe(double power, double inches, Direction direction) {

        double COUNTS_PER_MOTOR_REV = 2150.8;    // eg: TETRIX Motor Encoder
        double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
        double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        // Determine new target position, and pass to motor controller
        backLeftTarget = (int)(inches * COUNTS_PER_INCH);
        backRightTarget = (int)(inches * COUNTS_PER_INCH);
        frontLeftTarget = (int)(inches * COUNTS_PER_INCH);
        frontRightTarget = (int)(inches * COUNTS_PER_INCH);

        //Direction configuration (V2)
        if (direction == Direction.LEFT) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        } if (direction == Direction.RIGHT) {
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            telemetry.addLine("Error: No valid direction");
            stop();
        }

        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);


        // Turn On RUN_TO_POSITION
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion
        backLeft.setPower(Math.abs(power));
        backRight.setPower(Math.abs(power));
        frontLeft.setPower(Math.abs(power));
        frontRight.setPower(Math.abs(power));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that ALL motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy())) {
            // Display it for the driver.
            telemetry.addData("Running to", " %7d :%7d", backLeftTarget, backRightTarget, frontLeftTarget, frontRightTarget);
            telemetry.addData("Currently at", " at %7d :%7d", backLeft.getCurrentPosition(), backRight.getCurrentPosition(), frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn off RUN_TO_POSITION
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);

    }
}
