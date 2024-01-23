package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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


@Autonomous (name="Blue_Right")

public class BVBlueRight extends LinearOpMode {

    final String CURRENTOPMODE = "Blue_RIGHT";

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
    final Scalar LOW_RED = new Scalar(0, 0, 0);
    final Scalar HIGH_RED = new Scalar(0, 0, 0);

    //HSV values to use
    final Scalar LOW_HSV_VALUE = LOW_BLUE;
    final Scalar HIGH_HSV_VALUE = HIGH_BLUE;

    List<MatOfPoint> foundContours = new ArrayList<>();

    //Stores the converted RGB to HSV Mat
    Mat hsvMat = new Mat();
    //Stores a 'bitmap' of the values in range of color
    Mat inRangeMat = new Mat();
    //Designs how the morph var is stored
    Mat kernel = Mat.ones(7, 7, CvType.CV_8UC1);
    //Stores the morphed Mat which has most sound removed
    Mat morph = new Mat();
    //Stores the information of a contours' image topology, unused
    Mat hierarchy = new Mat();
    //contourMinimum range for the teleop controls
    int contourMinimum = 10000;

    //Vars for camera geometry + resolution
    int camWidth = 800;
    int camHeight = 600;

    //Variables for detection regions
    Rect leftRegion = new Rect(0, 0, 300, 600);
    Rect rightRegion = new Rect(300, 0, 700, 600);

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

    // Hardware declarations
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("WAIT FOR CAMERA TO INITIALIZE!!!!");
        telemetry.update();

        // Set Hardware behavior
        frontLeft = hardwareMap.dcMotor.get("FrontLeft");
        backLeft = hardwareMap.dcMotor.get("BackLeft");
        frontRight = hardwareMap.dcMotor.get("FrontRight");
        backRight = hardwareMap.dcMotor.get("BackRight");
        DcMotor SpinTake = hardwareMap.get(DcMotor.class, "CookieMonster");


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

        OpenCvWebcam webcam;
        OpenCvPipeline CVProcessor = new OpenCvPipeline() {

            @Override
            public Mat processFrame(Mat input) {

                //Converts all color from RGB to HSV
                Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

                //Creates a bitmap based on if the color is within the two scalar values
                Core.inRange(hsvMat, LOW_HSV_VALUE, HIGH_HSV_VALUE, inRangeMat);

                //Removes excess sound to create contours easily
                Imgproc.morphologyEx(inRangeMat, morph, Imgproc.MORPH_CLOSE, kernel);
                Imgproc.morphologyEx(morph, morph, Imgproc.MORPH_OPEN, kernel);

                //Creates a list (array) of contours based on the now morphed image
                List<MatOfPoint> contours = new ArrayList<>();

                Imgproc.findContours(morph, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

                foundContours = contours;

                //Draws rectangles for visual purposes
                Imgproc.rectangle(input, leftRegion, PURPLE, 5);
                Imgproc.rectangle(input, rightRegion, YELLOW, 5);

                telemetry.addLine("Looking for contours in ");
                telemetry.addData("",CURRENTOPMODE);

                for (int i = 0; i < contours.size(); i++) {

                    Rect rect = Imgproc.boundingRect(contours.get(i));
                    Point contourCent = new Point(((rect.width - rect.x) / 2.0) + rect.x, ((rect.height - rect.y) / 2.0) + rect.y);

                    //Comment out the if then statement below to draw all contours
                    //Note that all contours are detected in telemetry regardless


                    if (Math.abs(Imgproc.contourArea(foundContours.get(i))) > contourMinimum) {

                        Imgproc.drawContours(input, foundContours, i, GREEN, 5, 2);
                        Imgproc.drawMarker(input, contourCent, PURPLE, Imgproc.MARKER_TILTED_CROSS, 5);

                        Imgproc.rectangle(input, rect, GREEN);
                        Point contourCenter = new Point(((rect.br().x - rect.tl().x) / 2.0) + rect.tl().x, ((rect.br().y - rect.tl().y) / 2.0) + rect.tl().y);

                        telemetry.addData("points",contourCenter.x);

                        if(leftRegion.contains(contourCenter)) {
                            spikeLocation = elementLocation.LEFT;
                            leftCounts++;
                            break;

                        }
                        else if(rightRegion.contains(contourCenter)) {
                            spikeLocation = elementLocation.MIDDLE;
                            middleCounts++;
                            break;
                        }

                    }
                    //Extra else statement in order to view contours that are out of range
                    else {
                        Imgproc.drawContours(input, foundContours, i, RED, 5, 2);
                        spikeLocation = elementLocation.RIGHT;
                        rightCounts++;
                    }

                }

                if (spikeLocation== elementLocation.LEFT) {telemetry.addLine("Left");}
                if (spikeLocation== elementLocation.MIDDLE) {telemetry.addLine("Middle");}
                if (spikeLocation== elementLocation.RIGHT) {telemetry.addLine("Right");}

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
                telemetry.addLine("CAMERA INITIALIZED, You may start");
                telemetry.update();

            }

            public void onError(int errorCode) {
                telemetry.addData("ERROR UPON INITIALIZATION:", errorCode);
                telemetry.update();
            }

        });

        webcam.setPipeline(CVProcessor);

        waitForStart();

        webcam.stopStreaming();

        //spikeLocation = elementLocation.MIDDLE;
        //telemetry.addLine("FORCING MIDDLE SPIKE");

        while (opModeIsActive()) {

            telemetry.addLine("Prop found at");

            if (spikeLocation == elementLocation.LEFT) {

                telemetry.addLine("Left");

                driveAction(.6, 0.25, BVBlueRight.Direction.RIGHT, BVBlueRight.Rotate.NO);

                SpinTake.setPower(-0.5);

                driveAction(.6, 1, BVBlueRight.Direction.FORWARD, BVBlueRight.Rotate.NO);

                SpinTake.setPower(0);

                driveAction(.6, 6, BVBlueRight.Direction.FORWARD, BVBlueRight.Rotate.NO);
                driveAction(.6, 6, BVBlueRight.Direction.LEFT, BVBlueRight.Rotate.YES);

                SpinTake.setPower(0.45);

                driveAction(.6, 1.5, BVBlueRight.Direction.BACKWARD, BVBlueRight.Rotate.NO);

                SpinTake.setPower(0);

                driveAction(.3, 1.15, BVBlueRight.Direction.FORWARD, BVBlueRight.Rotate.NO);

            }
            if (spikeLocation == elementLocation.MIDDLE) {

                telemetry.addLine("Right");

                driveAction(.6, 0.25, BVBlueRight.Direction.RIGHT, BVBlueRight.Rotate.NO);

                driveAction(.6, 7.5, BVBlueRight.Direction.FORWARD, BVBlueRight.Rotate.NO);

            }
            if (spikeLocation == elementLocation.RIGHT) {

                telemetry.addLine("Middle");

                driveAction(.6, 0.25, BVBlueRight.Direction.RIGHT, BVBlueRight.Rotate.NO);

                SpinTake.setPower(-0.5);

                driveAction(.6, 1, BVBlueRight.Direction.FORWARD, BVBlueRight.Rotate.NO);

                SpinTake.setPower(0);

                driveAction(.6, 6, BVBlueRight.Direction.FORWARD, BVBlueRight.Rotate.NO);
                driveAction(.6, 6, BVBlueRight.Direction.RIGHT, BVBlueRight.Rotate.YES);

                SpinTake.setPower(0.45);

                driveAction(.6, 1.5, BVBlueRight.Direction.BACKWARD, BVBlueRight.Rotate.NO);

                SpinTake.setPower(0);

                driveAction(.3, 2.5, BVBlueRight.Direction.FORWARD, BVBlueRight.Rotate.NO);

            }
            if (spikeLocation == elementLocation.UNDECLARED) {
                telemetry.addLine("Undeclared");
            }
            telemetry.addData("L", leftCounts);
            telemetry.addData("M", middleCounts);
            telemetry.addData("R", rightCounts);

            telemetry.update();
            sleep(30000);
            // eject purple pixel, will need to declare intake hardware before implmenting

            // Move to backdrop and lift arm

            // position at backdrop, need to make another if/else tree

            // release pixel and move (if needed)

        }


    }

    public void driveAction(double power, double inches, BVBlueRight.Direction direction, BVBlueRight.Rotate rotateandmove) {

        boolean rotate = rotateandmove != BVBlueRight.Rotate.NO;

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
            if (direction == BVBlueRight.Direction.FORWARD && !rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            } if (direction == BVBlueRight.Direction.BACKWARD && !rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            } if (direction == BVBlueRight.Direction.LEFT && !rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            } if (direction == BVBlueRight.Direction.RIGHT && !rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            } if (direction == BVBlueRight.Direction.LEFT && rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            } if (direction == BVBlueRight.Direction.RIGHT && rotate) {
                backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            } if (direction == BVBlueRight.Direction.FORWARD && rotate) {
                telemetry.addLine("Used rotation == TRUE without an allowable direction;");
                telemetry.addLine("Stopped robot automatically.");
                telemetry.update();
                stop();
            } if (direction == BVBlueRight.Direction.BACKWARD && rotate) {
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

        }
    }
}
