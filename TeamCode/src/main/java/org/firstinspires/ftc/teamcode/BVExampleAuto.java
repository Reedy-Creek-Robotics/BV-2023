package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp
public class BVExampleAuto extends LinearOpMode {

    //RGB
    final Scalar RED = new Scalar(255, 0, 0);
    final Scalar GREEN = new Scalar(0, 255, 0);
    final Scalar PURPLE = new Scalar(255, 0, 255);
    final Scalar YELLOW = new Scalar(255, 255, 0);

    //HSV Blue
    final Scalar LOW_BLUE = new Scalar(100, 100, 100);
    final Scalar HIGH_BLUE = new Scalar(130, 255, 255);

    //--------------------------------------------------------

    List<MatOfPoint> contoursBlue = new ArrayList<>();

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



    //--------------------------------------------------------

    //Vars for camera geometry + resolution

    int camWidth = 800;
    int camHeight = 600;

    Rect rect1 = new Rect(0, 0, 300, 600);
    Rect rect2 = new Rect(300, 0, 700, 600);

    enum elementLocation {
        LEFT,
        MIDDLE,
        RIGHT,
        UNDECLARED
    }
    elementLocation spikeLocation = elementLocation.UNDECLARED;


    //--------------------------------------------------------



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

    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    public void runOpMode() throws InterruptedException {
        ////////////////////////
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

        ////////////////////////

        OpenCvWebcam webcam;

        OpenCvPipeline blueProcessor = new OpenCvPipeline() {

            @Override
            public Mat processFrame(Mat input) {

                //Converts all color from RGB to HSV
                Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

                //Creates a bitmap based on if the color is within the two scalar values
                Core.inRange(hsvMat, LOW_BLUE, HIGH_BLUE, inRangeMat);

                //Removes excess sound to create contours easily
                Imgproc.morphologyEx(inRangeMat, morph, Imgproc.MORPH_CLOSE, kernel);
                Imgproc.morphologyEx(morph, morph, Imgproc.MORPH_OPEN, kernel);

                //Creates a list (array) of contours based on the now morphed image
                List<MatOfPoint> contours = new ArrayList<>();

                Imgproc.findContours(morph, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

                BVExampleAuto.this.contoursBlue = contours;

                //Draws rectangles for visual purposes
                Imgproc.rectangle(input, rect1, PURPLE, 5);
                Imgproc.rectangle(input, rect2, YELLOW, 5);

                for (int i = 0; i < contours.size(); i++) {

                    Rect rect = Imgproc.boundingRect(contours.get(i));
                    Point contourCent = new Point(((rect.width - rect.x) / 2.0) + rect.x, ((rect.height - rect.y) / 2.0) + rect.y);

                    //Comment out the if then statement below to draw all contours
                    //Note that all contours are detected in telemetry regardless
                    if (Math.abs(Imgproc.contourArea(contoursBlue.get(i))) > contourMinimum) {

                        Imgproc.drawContours(input, contoursBlue, i, GREEN, 5, 2);
                        Imgproc.drawMarker(input, contourCent, PURPLE, Imgproc.MARKER_TILTED_CROSS, 5);

                        Imgproc.rectangle(input, rect, GREEN);
                        Point contourCentBlue = new Point(((rect.br().x - rect.tl().x) / 2.0) + rect.tl().x, ((rect.br().y - rect.tl().y) / 2.0) + rect.tl().y);
                        //rect2.contains(contourCent)
                        telemetry.addLine("Here2");
                        spikeLocation = elementLocation.RIGHT;
                        if(rect1.contains(contourCentBlue)) {
                            spikeLocation = elementLocation.LEFT;
                            //telemetry.addLine("Left");

                        }
                        else if(rect2.contains(contourCentBlue)) {
                            spikeLocation = elementLocation.MIDDLE;
                            //telemetry.addLine("Middle");
                        }

                        if(spikeLocation == elementLocation.RIGHT){
                            spikeLocation = elementLocation.RIGHT;
                            //telemetry.addLine("Right");
                        }
                        //telemetry.update();
                    }
                    //Extra else statement in order to view contours that are out of range
                    else {
                        Imgproc.drawContours(input, contoursBlue, i, RED, 5, 2);
                    }
                }

                //Returns input to webcam
                return input;
            }
        };

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                telemetry.addLine("INTIALIZATION SUCCESSFUL");
                telemetry.update();

                webcam.startStreaming(camWidth, camHeight, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("ERROR UPON INITIALIZATION:", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("KEEP THE ROBOT NEAR THE LEFT SIDE OF THE SQUARE");

        /*while (opModeInInit()) {
            telemetry.addLine("KEEP THE ROBOT NEAR THE LEFT SIDE OF THE SQUARE");
        }*/
        webcam.setPipeline(blueProcessor);
        waitForStart();

        while (opModeIsActive()) {


            List<MatOfPoint> contoursBlue = BVExampleAuto.this.contoursBlue;




            telemetry.addLine("here");

            if (spikeLocation == elementLocation.LEFT) {
                telemetry.addLine("Left");
            }
            if (spikeLocation == elementLocation.RIGHT) {
                telemetry.addLine("Right");
            }
            if (spikeLocation == elementLocation.MIDDLE) {
                telemetry.addLine("Middle");
            }
            if (spikeLocation == elementLocation.UNDECLARED) {
                telemetry.addLine("Undeclared");
            }
            telemetry.update();
            sleep(30000);

            telemetry.addLine("KEEP THE ROBOT NEAR THE LEFT SIDE OF THE SQUARE");
            telemetry.addLine("Detecting BLUE Contours");
            telemetry.addData("Webcam pipeline activity", webcam.getPipelineTimeMs());
            telemetry.addData("Contours Detected", contoursBlue.size());
            telemetry.addData("Contour Minimum Vision", contourMinimum);



            for (int i = 0; i < contoursBlue.size(); i++) {

                //If then statement to clear out unnecessary contours
                if (Math.abs(Imgproc.contourArea(contoursBlue.get(i))) > contourMinimum) {

                    Rect rect = Imgproc.boundingRect(contoursBlue.get(i));
                    Point contourCent = new Point(((rect.br().x - rect.tl().x) / 2.0) + rect.tl().x, ((rect.br().y - rect.tl().y) / 2.0) + rect.tl().y);

                    Point rectTl = new Point(rect.tl().x, rect.tl().y);
                    Point rectBr = new Point(rect.br().x, rect.br().y);

                    telemetry.addData("Center point", contourCent);
                    telemetry.addData("Top Left Rect", rectTl);
                    telemetry.addData("Bottom Right Rect", rectBr);

                    telemetry.addData("Element area", Imgproc.contourArea(contoursBlue.get(i)));

                    if (rect1.contains(contourCent)) {
                        spikeLocation = elementLocation.LEFT;
                    } else if (rect2.contains(contourCent)) {
                        spikeLocation = elementLocation.MIDDLE;
                    }
                    else {
                        spikeLocation = elementLocation.RIGHT;
                    }
                } else {
                    telemetry.addData("Non-Element Contour Area", Imgproc.contourArea(contoursBlue.get(i)));
                }
            }

            if (spikeLocation == elementLocation.RIGHT) {
                telemetry.addLine("Element on NO Rectangle / RIGHT spike");
            } else if (spikeLocation == elementLocation.LEFT) {
                telemetry.addLine("Element on LEFT Rectangle / LEFT spike");
            } else if (spikeLocation == elementLocation.MIDDLE) {
                telemetry.addLine("Element on RIGHT Rectangle / MIDDLE spike");
            }

            telemetry.update();
            ///////////////////////////////////////////////////
            // Any Common move action before going to spike?
            driveAction(.6, 2, Direction.FORWARD, Rotate.NO);
            sleep(1000);
            // move to spike
            if (spikeLocation == elementLocation.RIGHT) {
                driveAction(.6, 5, Direction.RIGHT, Rotate.YES);
                telemetry.addLine("Executing Right");
            } else if (spikeLocation == elementLocation.MIDDLE) {
                driveAction(.6, 1, Direction.FORWARD, Rotate.NO);
                telemetry.addLine("Executing middle");
            } else {
                driveAction(.6, 5, Direction.LEFT, Rotate.YES);
                telemetry.addLine("Executing left");
            }
            telemetry.update();
            sleep(5000);

            // eject purple pixel, will need to declare intake hardware before implmenting

            // Move to backdrop and lift arm

            // position at backdrop, need to make another if/else tree

            // release pixel and move (if needed)


        }


    }

    public void driveAction(double power, double inches, Direction direction, Rotate rotateandmove) {

        boolean rotate = rotateandmove != Rotate.NO;

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

        }
    }
}
