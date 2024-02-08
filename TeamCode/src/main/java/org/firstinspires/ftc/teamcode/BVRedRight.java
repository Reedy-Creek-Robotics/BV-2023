package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;


@Autonomous (name="Red_Right")

public class BVRedRight extends LinearOpMode {

    final String CURRENTOPMODE = "Red_RIGHT";

    // Declare Image Processing constants and variables
    //RGB
    final Scalar RED = new Scalar(255, 0, 0);
    final Scalar GREEN = new Scalar(0, 255, 0);
    final Scalar PURPLE = new Scalar(255, 0, 255);
    final Scalar YELLOW = new Scalar(255, 255, 0);

    //HSV Blue
    final Scalar LOW_BLUE = new Scalar(100, 100, 100);
    final Scalar HIGH_BLUE = new Scalar(130, 255, 255);

    //HSV Red
    final Scalar LOW_RED1 = new Scalar(248, 100, 100);
    final Scalar HIGH_RED1 = new Scalar(0, 255, 255);

    final Scalar LOW_RED2 = new Scalar(0, 100, 100);
    final Scalar HIGH_RED2 = new Scalar(12, 255, 255);

    //HSV values to use
    final Scalar LOW_HSV_VALUE1 = LOW_RED1;
    final Scalar HIGH_HSV_VALUE1 = HIGH_RED1;

    final Scalar LOW_HSV_VALUE2 = LOW_RED2;
    final Scalar HIGH_HSV_VALUE2 = HIGH_RED2;

    List<MatOfPoint> foundContours = new ArrayList<>();

    //Stores the converted RGB to HSV Mat
    Mat hsvMat1 = new Mat();
    Mat hsvMat2 = new Mat();
    //Stores a 'bitmap' of the values in range of color
    Mat inRangeMat1 = new Mat();
    Mat inRangeMat2 = new Mat();
    //Designs how the morph var is stored
    Mat kernel = Mat.ones(7, 7, CvType.CV_8UC1);
    //Stores the morphed Mat which has most sound removed
    Mat morph1 = new Mat();
    Mat morph2 = new Mat();
    //Merged Mat of both morph1 and morph2
    Mat merge = new Mat();
    //Stores the information of a contours' image topology, unused
    Mat hierarchy = new Mat();
    //contourMinimum range for the teleop controls
    int contourMinimum = 10000;

    List<MatOfPoint> contoursRed = new ArrayList<>();

    //Vars for camera geometry + resolution
    int camWidth = 800;
    int camHeight = 600;

    //Variables for detection regions
    Rect leftRegion = new Rect(0, 0, 600, 600);
    Rect rightRegion = new Rect(600, 0, 400, 600);

    //needed enums
    enum elementLocation {
        LEFT,
        MIDDLE,
        RIGHT,
        UNDECLARED
    }

    enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }

    enum Rotate {
        YES,
        NO
    }

    // End result of element location detection
    elementLocation spikeLocation = elementLocation.UNDECLARED;
    int leftCounts = 0;
    int rightCounts = 0;
    int middleCounts = 0;

    //Saved doubles for claw positions
    double OpenClaw = 0.14;
    double ClosedClaw = 0;

    // Hardware declarations
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor SpinTake;
    Servo ClawRotation;
    Servo Claw;
    DcMotor Slide;



    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("WAIT FOR CAMERA TO INITIALIZE!!!!");
        telemetry.update();

        // Set Hardware behavior
        frontLeft = hardwareMap.dcMotor.get("FrontLeft");
        backLeft = hardwareMap.dcMotor.get("BackLeft");
        frontRight = hardwareMap.dcMotor.get("FrontRight");
        backRight = hardwareMap.dcMotor.get("BackRight");
        SpinTake = hardwareMap.get(DcMotor.class, "CookieMonster");
        ClawRotation = hardwareMap.get(Servo.class, "ClawRotation");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setDirection(Servo.Direction.REVERSE);
        Slide = hardwareMap.get(DcMotor.class, "Slide");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Claw.setPosition(0);
        ClawRotation.setPosition(0);
        Slide.setTargetPosition(-50);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(.6);

        while (Slide.isBusy()) {

        }

        OpenCvWebcam webcam;
        OpenCvPipeline CVProcessor = new OpenCvPipeline() {

            @Override
            public Mat processFrame(Mat input) {

                //Converts all color from RGB to HSV
                Imgproc.cvtColor(input, hsvMat1, Imgproc.COLOR_RGB2HSV);
                Imgproc.cvtColor(input, hsvMat2, Imgproc.COLOR_RGB2HSV);

                //Creates a bitmap based on if the color is within the two scalar values
                Core.inRange(hsvMat1, LOW_HSV_VALUE1, HIGH_HSV_VALUE1, inRangeMat1);
                Core.inRange(hsvMat2, LOW_HSV_VALUE2, HIGH_HSV_VALUE2, inRangeMat2);

                //Removes excess sound to create contours easily
                Imgproc.morphologyEx(inRangeMat1, morph1, Imgproc.MORPH_CLOSE, kernel);
                Imgproc.morphologyEx(morph1, morph1, Imgproc.MORPH_OPEN, kernel);

                Imgproc.morphologyEx(inRangeMat2, morph2, Imgproc.MORPH_CLOSE, kernel);
                Imgproc.morphologyEx(morph2, morph2, Imgproc.MORPH_OPEN, kernel);

                Core.bitwise_or(morph1, morph2, merge);

                //Creates a list (array) of contours based on the now morphed image
                List<MatOfPoint> contours = new ArrayList<>();

                Imgproc.findContours(merge, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

                foundContours = contours;

                //Draws rectangles for visual purposes
                Imgproc.rectangle(input, leftRegion, PURPLE, 5);
                Imgproc.rectangle(input, rightRegion, YELLOW, 5);

                for (int i = 0; i < contours.size(); i++) {

                    Rect rect = Imgproc.boundingRect(contours.get(i));
                    Point contourCent = new Point(((rect.width - rect.x) / 2.0) + rect.x, ((rect.height - rect.y) / 2.0) + rect.y);

                    if (Math.abs(Imgproc.contourArea(foundContours.get(i))) > contourMinimum) {

                        Imgproc.drawContours(input, foundContours, i, GREEN, 5, 2);
                        Imgproc.drawMarker(input, contourCent, PURPLE, Imgproc.MARKER_TILTED_CROSS, 5);

                        Imgproc.rectangle(input, rect, GREEN);
                        Point contourCenter = new Point(((rect.br().x - rect.tl().x) / 2.0) + rect.tl().x, ((rect.br().y - rect.tl().y) / 2.0) + rect.tl().y);

                        telemetry.addData("points", contourCenter.x);

                        if (leftRegion.contains(contourCenter)) {
                            spikeLocation = elementLocation.MIDDLE;
                            middleCounts++;
                            break;

                        }
                        else if (rightRegion.contains(contourCenter)) {
                            spikeLocation = elementLocation.RIGHT;
                            rightCounts++;
                            break;
                        }

                    }
                    //Extra else statement in order to view contours that are out of range
                    else {
                        Imgproc.drawContours(input, foundContours, i, RED, 5, 2);
                        spikeLocation = elementLocation.LEFT;
                        leftCounts++;
                    }

                }

                if (spikeLocation == elementLocation.LEFT) {telemetry.addLine("Left");}
                if (spikeLocation == elementLocation.MIDDLE) {telemetry.addLine("Middle");}
                if (spikeLocation == elementLocation.RIGHT) {telemetry.addLine("Right");}

                telemetry.addLine("CAMERA INITIALIZED");

                telemetry.update();

                return input;
            }
        };

        ////////////////////////
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            public void onOpened() {

                webcam.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);

            }

            public void onError(int errorCode) {
                telemetry.addData("ERROR UPON INITIALIZATION:", errorCode);
                telemetry.update();
            }

        });

        webcam.setPipeline(CVProcessor);

        waitForStart();

        webcam.stopStreaming();

        while (opModeIsActive()) {

            ClawRotation.setPosition(0);

            telemetry.addData("Prop found at", spikeLocation);

            if (spikeLocation == elementLocation.LEFT) {

                //Spike Navigation
                SpinTake.setPower(-0.5);
                driveAction(.6, 8, Direction.FORWARD, Rotate.NO);
                SpinTake.setPower(0);
                driveAction(.6, 6.25, Direction.LEFT, Rotate.YES);
                SpinTake.setPower(0.4);
                driveAction(.6, 1.5, Direction.BACKWARD, Rotate.NO);
                SpinTake.setPower(0);
                driveAction(.3, 2.65, Direction.FORWARD, Rotate.NO);
                driveAction(.6, 9.05, Direction.BACKWARD, Rotate.NO);
                driveAction(.6, 1, Direction.RIGHT, Rotate.NO);

                //Forwards is backwards for the slide, therefore use negative integers past 350.
                Slide.setTargetPosition(-550);
                Slide.setPower(.6);

                while (Slide.isBusy()) {

                }

                ClawRotation.setPosition(.275);

                driveAction(.6, 2.15, Direction.BACKWARD, Rotate.NO);
                Claw.setPosition(.14);
                driveAction(.3, 2, Direction.FORWARD, Rotate.NO);

                Claw.setPosition(0);
                ClawRotation.setPosition(0);

                Slide.setTargetPosition(-50);
                Slide.setPower(.6);

                while (Slide.isBusy()) {

                }

                driveAction(.6, 9, Direction.LEFT, Rotate.NO);
                driveAction(.6, 6.25, Direction.RIGHT, Rotate.YES);

            }

            if (spikeLocation == elementLocation.MIDDLE) {

                driveAction(.6, 7.35, Direction.FORWARD, Rotate.NO);

                //Blackboard Navigation
                driveAction(.6, 1.5, Direction.BACKWARD, Rotate.NO);
                driveAction(.6, 6.25, Direction.LEFT, Rotate.YES);
                driveAction(.6, 8, Direction.BACKWARD, Rotate.NO);
                driveAction(.6, 1.5, Direction.RIGHT, Rotate.NO);

                //Forwards is backwards for the slide, therefore use negative integers past 350.
                Slide.setTargetPosition(-550);
                Slide.setPower(.6);

                while (Slide.isBusy()) {

                }

                ClawRotation.setPosition(.275);

                driveAction(.6, 2.15, Direction.BACKWARD, Rotate.NO);
                Claw.setPosition(.14);
                driveAction(.3, 2, Direction.FORWARD, Rotate.NO);

                Claw.setPosition(0);
                ClawRotation.setPosition(0);

                Slide.setTargetPosition(-50);
                Slide.setPower(.6);

                while (Slide.isBusy()) {

                }

                driveAction(.6, 6.5, Direction.LEFT, Rotate.NO);
                driveAction(.6, 6.25, Direction.RIGHT, Rotate.YES);

            }

            if (spikeLocation == elementLocation.RIGHT) {

                //Spike Navigation
                SpinTake.setPower(-0.5);
                driveAction(.6, 1, Direction.FORWARD, Rotate.NO);
                SpinTake.setPower(0);
                driveAction(.6, 6, Direction.RIGHT, Rotate.NO);
                driveAction(.6, 6.5, Direction.FORWARD, Rotate.NO);
                driveAction(.6, 6.25, Direction.LEFT, Rotate.YES);
                SpinTake.setPower(0.4);
                driveAction(.6, 1.8, Direction.BACKWARD, Rotate.NO);
                SpinTake.setPower(0);
                driveAction(.3, 2.5, Direction.FORWARD, Rotate.NO);

                driveAction(.6, 3.85, Direction.BACKWARD,  Rotate.NO);
                driveAction(.6, 1.75, Direction.LEFT, Rotate.NO);

                //Forwards is backwards for the slide, therefore use negative integers past 350.
                Slide.setTargetPosition(-550);
                Slide.setPower(.6);

                sleep(1500);

                ClawRotation.setPosition(.275);

                driveAction(.6, 2.15, Direction.BACKWARD, Rotate.NO);
                Claw.setPosition(.14);
                driveAction(.3, 2, Direction.FORWARD, Rotate.NO);

                Claw.setPosition(0);
                ClawRotation.setPosition(0);

                Slide.setTargetPosition(-50);
                Slide.setPower(.6);

                while (Slide.isBusy()) {

                }

                driveAction(.6, 5, Direction.LEFT, Rotate.NO);
                driveAction(.6, 6.25, Direction.RIGHT, Rotate.YES);


            }

            telemetry.addData("L", leftCounts);
            telemetry.addData("M", middleCounts);
            telemetry.addData("R", rightCounts);
            telemetry.update();

            sleep(30000);

        }


    }

    public void driveAction(double power, double inches, BVRedRight.Direction direction, BVRedRight.Rotate rotateandmove) {

        boolean rotate = rotateandmove != BVRedRight.Rotate.NO;

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
            double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

            // Determine new target position, and pass to motor controller
            downLeftTarget = (int)(inches * COUNTS_PER_INCH);
            downRightTarget = (int)(inches * COUNTS_PER_INCH);
            upLeftTarget = (int)(inches * COUNTS_PER_INCH);
            upRightTarget = (int)(inches * COUNTS_PER_INCH);

            //Direction configuration
            if (direction == BVRedRight.Direction.FORWARD && !rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            } if (direction == BVRedRight.Direction.BACKWARD && !rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            } if (direction == BVRedRight.Direction.LEFT && !rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            } if (direction == BVRedRight.Direction.RIGHT && !rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            } if (direction == BVRedRight.Direction.LEFT && rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            } if (direction == BVRedRight.Direction.RIGHT && rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            } if (direction == BVRedRight.Direction.FORWARD && rotate) {
                telemetry.addLine("Used rotation == TRUE without an allowable direction;");
                telemetry.addLine("Stopped robot automatically.");
                telemetry.update();
                stop();
            } if (direction == BVRedRight.Direction.BACKWARD && rotate) {
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
                    ((backLeft.isBusy() || frontLeft.isBusy()) && (backRight.isBusy() || frontRight.isBusy()))) {
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

        }
    }
}
