package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class BVAutoEncoder extends LinearOpMode {

    //Variable declarations
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;

    double y;
    double x;
    double rx;
    double denominator;
    double limiter = 0.6;
    double frontLeft;
    double backLeft;
    double frontRight;
    double backRight;
    double constant;
    int inches;
    double idle = 0.25;
    boolean directionLimiter = false;
    boolean rotationLimiter = false;

    String rotate;
    String direction;

    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("BackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
        motorBackRight = hardwareMap.dcMotor.get("BackRight");
        //Autonomous is not in driver-centric, therefore this teleOp isn't either

        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeft = ((y + x + rx) / denominator);
            backLeft = ((y - x + rx) / denominator);
            frontRight = ((y - x - rx) / denominator);
            backRight = ((y + x - rx) / denominator);

            //Limiter input
            if (gamepad1.dpad_down && timer.milliseconds() > 350) {
                limiter = 0.3;
                timer.reset();
            } if (gamepad1.dpad_up && timer.milliseconds() > 350) {
                limiter = 0.6;
                timer.reset();
            }

            //Positive maximum
            if (frontLeft > limiter) {
                frontLeft = limiter;
            } if (frontRight > limiter) {
                frontRight = limiter;
            } if (backLeft > limiter) {
                backLeft = limiter;
            } if (backRight > limiter) {
                backRight = limiter;
            }

            //Negative minimum
            if (frontLeft < -limiter) {
                frontLeft = -limiter;
            } if (frontRight < -limiter) {
                frontRight = -limiter;
            } if (backLeft < -limiter) {
                backLeft = -limiter;
            } if (backRight < -limiter) {
                backRight = -limiter;
            }

            constant = (Math.abs(motorFrontRight.getCurrentPosition()) +
                       Math.abs(motorBackRight.getCurrentPosition())) -
                       (Math.abs(motorFrontLeft.getCurrentPosition()) +
                       Math.abs(motorFrontLeft.getCurrentPosition()));

            inches = (int)(constant * (2150.8 / 4 * 3.1415));

            //If statements below are broken in parenthesis for readability
            if (!directionLimiter) {
                if ((y > idle) && (-idle < x && x < idle)) {
                    direction = "forward";
                    directionLimiter = true;
                } if ((y < -idle) && (-idle < x && x < idle)) {
                    direction = "backward";
                    directionLimiter = true;
                } if ((-idle < y && y < idle) && (x < -idle)) {
                    direction = "left";
                    directionLimiter = true;
                } if ((-idle < y && y < idle) && (x > idle)) {
                    direction = "right";
                    directionLimiter = true;
                }
            }

            if (!rotationLimiter) {
                if ((-idle < rx && rx < idle)) {
                    rotate = "Linearly";
                    rotationLimiter = true;
                } if ((idle < rx && rx < -idle)) {
                    rotate = "Rotating";
                    rotationLimiter = true;
                }
            }

            //Resets calculations + telemetry
            if (gamepad1.x) {
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                directionLimiter = false;
                rotationLimiter = false;
                telemetry.update();
            }

            telemetry.addData("Front Left Pos:", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Front Right Pos:", motorFrontRight.getCurrentPosition());
            telemetry.addData("Back Left Pos:", motorBackLeft.getCurrentPosition());
            telemetry.addData("Back Right Pos:", motorBackRight.getCurrentPosition());
            telemetry.addData("Inches:", inches);
            telemetry.addData("Direction:", rotate + direction);
            telemetry.update();

            motorFrontLeft.setPower(frontLeft);
            motorBackLeft.setPower(backLeft);
            motorFrontRight.setPower(frontRight);
            motorBackRight.setPower(backRight);

        }
    }
}
