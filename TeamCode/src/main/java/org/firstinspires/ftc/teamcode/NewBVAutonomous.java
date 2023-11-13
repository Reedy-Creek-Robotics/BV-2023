package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class NewBVAutonomous extends LinearOpMode {

    enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    DcMotor frontLeft = hardwareMap.dcMotor.get("FrontLeft");
    DcMotor backLeft = hardwareMap.dcMotor.get("BackLeft");
    DcMotor frontRight = hardwareMap.dcMotor.get("FrontRight");
    DcMotor backRight = hardwareMap.dcMotor.get("BackRight");

    @Override
    public void runOpMode() throws InterruptedException {

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Starting at",  "%7d :%7d",
                    backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition(),
                    frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition());
            telemetry.update();

            // Motor action here
            // IF ROTATION == TRUE, ROTATE LEFT OR RIGHT ONLY



            telemetry.addLine("Path Complete");
            telemetry.update();

        }
    }

    public void motorAction(double speed, double inches, Direction direction, boolean rotate) {
        int downLeftTarget;
        int downRightTarget;
        int upLeftTarget;
        int upRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Calculate the COUNTS_PER_INCH for your specific drive train.
            // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
            // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
            // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
            // This is gearing DOWN for less speed and more torque.
            // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
            double COUNTS_PER_MOTOR_REV = 2150.8;    // eg: TETRIX Motor Encoder
            double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
            double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
            double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);

            // Determine new target position, and pass to motor controller
            downLeftTarget = backLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            downRightTarget = backRight.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            upLeftTarget = frontLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            upRightTarget = frontRight.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            //Direction configuration

            if (direction == Direction.FORWARD && !rotate) {
                backLeft.setTargetPosition(downLeftTarget);
                backRight.setTargetPosition(downRightTarget);
                frontLeft.setTargetPosition(upLeftTarget);
                frontRight.setTargetPosition(upRightTarget);
            } if (direction == Direction.BACKWARD && !rotate) {
                backLeft.setTargetPosition(-downLeftTarget);
                backRight.setTargetPosition(-downRightTarget);
                frontLeft.setTargetPosition(-upLeftTarget);
                frontRight.setTargetPosition(-upRightTarget);
            } if (direction == Direction.LEFT && !rotate) {
                backLeft.setTargetPosition(downLeftTarget);
                backRight.setTargetPosition(-downRightTarget);
                frontLeft.setTargetPosition(-upLeftTarget);
                frontRight.setTargetPosition(upRightTarget);
            } if (direction == Direction.RIGHT && !rotate) {
                backLeft.setTargetPosition(-downLeftTarget);
                backRight.setTargetPosition(downRightTarget);
                frontLeft.setTargetPosition(upLeftTarget);
                frontRight.setTargetPosition(-upRightTarget);
            } if (direction == Direction.LEFT && rotate) {
                backLeft.setTargetPosition(-downLeftTarget);
                backRight.setTargetPosition(downRightTarget);
                frontLeft.setTargetPosition(-upLeftTarget);
                frontRight.setTargetPosition(upRightTarget);
            } if (direction == Direction.RIGHT && rotate) {
                backLeft.setTargetPosition(downLeftTarget);
                backRight.setTargetPosition(-downRightTarget);
                frontLeft.setTargetPosition(upLeftTarget);
                frontRight.setTargetPosition(-upRightTarget);
            } if (direction == Direction.FORWARD && rotate) {
                telemetry.addLine("Used rotation == TRUE without an allowable direction;");
                telemetry.addLine("Stopped robot automatically.");
                telemetry.update();
                stop();
            } if (direction == Direction.BACKWARD && rotate) {
                telemetry.addLine("Used rotation == TRUE without an allowable direction;");
                telemetry.addLine("Stopped robot automatically.");
                telemetry.update();
                stop();
            }

            // Turn On RUN_TO_POSITION
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start motion
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that ALL motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", downLeftTarget, downRightTarget, upLeftTarget, upRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d", backLeft.getCurrentPosition(), backRight.getCurrentPosition(), frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);


            // Turn off RUN_TO_POSITION
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}