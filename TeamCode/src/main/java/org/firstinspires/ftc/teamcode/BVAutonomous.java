package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class BVAutonomous extends LinearOpMode {

    enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

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

            /* Motor action below
             4 Params in order: Power (double), Inches (double), Direction.[direction] (Enum), rotate (boolean)
             IF ROTATION == TRUE,  Direction.LEFT OR Direction.RIGHT ONLY
             Inches var is DOUBLE, not INT
             motorAction method runs actions sequentially automatically

            6 inches on 0.6 power = ~1 tile
            6 inches on 0.6 power = ~90 Degrees
            Linear driving is recommended over strafing!!

            Moves the robot forward at 0.6 power for 12 inches
            motorAction(0.6, 12.0, Direction.FORWARD, false);
            Rotates the robot to the left at 0.6 power for 0.5 inches
            motorAction(0.6, 6.0, Direction.LEFT, true);
            Strafes the robot to the right at 0.6 power for 12 inches
            motorAction(0.6, 12.0, Direction.RIGHT, false);*/

            motorAction(0.6,12.0, Direction.LEFT,false);


            telemetry.addLine("Path Complete");
            telemetry.update();

        }
    }

    public boolean webCamActive() {

        boolean element = false;

        //OpenCvWebcam webcam = this.webcam;
        //this.webcam = webcam;

        return element;
    }

    public void motorAction(double power, double inches, Direction direction, boolean rotate) {

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
            downLeftTarget = (int)(inches * COUNTS_PER_INCH);
            downRightTarget = (int)(inches * COUNTS_PER_INCH);
            upLeftTarget = (int)(inches * COUNTS_PER_INCH);
            upRightTarget = (int)(inches * COUNTS_PER_INCH);

            //Direction configuration
            if (direction == Direction.FORWARD && !rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            } if (direction == Direction.BACKWARD && !rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            } if (direction == Direction.LEFT && !rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            } if (direction == Direction.RIGHT && !rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            } if (direction == Direction.LEFT && rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            } if (direction == Direction.RIGHT && rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
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

            //Set targets
            backLeft.setTargetPosition(downLeftTarget);
            backRight.setTargetPosition(downRightTarget);
            frontLeft.setTargetPosition(upLeftTarget);
            frontRight.setTargetPosition(upRightTarget);

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
                telemetry.addData("Running to", " %7d :%7d", downLeftTarget, downRightTarget, upLeftTarget, upRightTarget);
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
            //backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}