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

@Autonomous
public class BVAutonomousBlueLeft extends LinearOpMode {

    //HSV Red
    //final Scalar LOW_RED1 = new Scalar(248, 100, 100);
    //final Scalar HIGH_RED1 = new Scalar(0, 255, 255);

    //final Scalar LOW_RED2 = new Scalar(0, 100, 100);
    //final Scalar HIGH_RED2 = new Scalar(12, 255, 255);

    //--------------------------------------------------------

    //Red Processor Vars
    //Two vars of each since we are creating two comparisons then merging them.
    //Refer to blue processor comments for descriptions of mats.

    //Mat hsvMat1 = new Mat();
    //Mat hsvMat2 = new Mat();

    //Mat inRangeMat1 = new Mat();
    //Mat inRangeMat2 = new Mat();

    //Mat morph1 = new Mat();
    //Mat morph2 = new Mat();

    Mat hierarchy = new Mat();

    Mat kernel = Mat.ones(7, 7, CvType.CV_8UC1);

    List<MatOfPoint> contoursRed = new ArrayList<>();

    //Mat merge = new Mat();

    //--------------------------------------------------------

    int contourMinimum = 10000;

    int count = 0;

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

    //HSV Blue
    final Scalar LOW_BLUE = new Scalar(100, 100, 100);
    final Scalar HIGH_BLUE = new Scalar(130, 255, 255);

    //RGB Scalar
    final Scalar GREEN = new Scalar(0, 255, 0);
    final Scalar PURPLE = new Scalar(255, 0, 255);
    final Scalar YELLOW = new Scalar(255, 255, 0);
    final Scalar RED = new Scalar(255, 0, 0);

    List<MatOfPoint> contoursBlue = new ArrayList<>();

    //Stores the converted RGB to HSV Mat
    Mat hsvMat = new Mat();
    //Stores a 'bitmap' of the values in range of color
    Mat inRangeMat = new Mat();
    //Stores the morphed Mat which has most sound removed
    Mat morph = new Mat();

    //--------------------------------------------------------

    OpenCvWebcam webcam;

    OpenCvPipeline blueProcessor;

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

        if (opModeInInit()) {
            telemetry.addLine("Keep robot on *R*ight side for *R*ed");
            telemetry.addLine("Keep robot on *L*eft side for B*l*ue");
            telemetry.update();
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(camWidth, camHeight, OpenCvCameraRotation.UPRIGHT);
                webcam.setPipeline(blueProcessor);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("ERROR UPON INITIALIZATION:", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {

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

            //motorAction(0.6,12.0, Direction.LEFT,false);

            /* Enum (officialSpikeLocation) is returned upon method call webCamActivateRed() or webCamActivateBlue(),

              Do NOT use both methods in one class, they both rely on the same enum. */

            /*elementLocation officialSpikeLocation = null;

            if (spikeLocation == elementLocation.UNDECLARED) {
                officialSpikeLocation = webCamActivateBlue();
            }

            telemetry.addLine("Detecting BLUE Contours");
            telemetry.addData("Blue Element Detected", officialSpikeLocation);
            telemetry.addData("Webcam pipeline activity", webcam.getPipelineTimeMs());
            telemetry.addData("Contours Detected", contoursBlue.size());
            telemetry.addData("Contour Minimum Vision", contourMinimum);*/

            motorAction(1, 12, Direction.LEFT, false);

            //telemetry.update();
        }
    }

    public elementLocation webCamActivateBlue() {

        blueProcessor = new OpenCvPipeline() {

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

                BVAutonomousBlueLeft.this.contoursBlue = contours;

                //Draws rectangles for visual purposes
                Imgproc.rectangle(input, rect1, PURPLE, 5);
                Imgproc.rectangle(input, rect2, YELLOW, 5);

                for (int i = 0; i < contours.size(); i++) {

                    Rect rect = Imgproc.boundingRect(contours.get(i));
                    Point contourCent = new Point(((rect.br().x - rect.tl().x) / 2.0) + rect.tl().x, ((rect.br().y - rect.tl().y) / 2.0) + rect.tl().y);

                    //Comment out the if then statement below to draw all contours
                    //Note that all contours are detected in telemetry regardless
                    if (Math.abs(Imgproc.contourArea(contoursBlue.get(i))) > contourMinimum) {

                        Imgproc.drawContours(input, contoursBlue, i, GREEN, 5, 2);
                        Imgproc.drawMarker(input, contourCent, PURPLE, Imgproc.MARKER_TILTED_CROSS, 5);

                        Imgproc.rectangle(input, rect, GREEN);
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

        waitForStart();

        webcam.setPipeline(blueProcessor);

        for (int i = 0; i < contoursBlue.size(); i++) {

            //If then statement to clear out unnecessary contours
            if (Math.abs(Imgproc.contourArea(contoursBlue.get(i))) > contourMinimum) {

                Rect rect = Imgproc.boundingRect(contoursBlue.get(i));
                Point contourCent = new Point((((rect.br().x - rect.tl().x) / 2.0) + rect.tl().x), (((rect.br().y - rect.tl().y) / 2.0) + rect.tl().y));

                /*Point rectTl = new Point(rect.tl().x, rect.tl().y);
                Point rectBr = new Point(rect.br().x, rect.br().y);

                telemetry.addData("Center point", contourCent);
                telemetry.addData("Top Left Rect", rectTl);
                telemetry.addData("Bottom Right Rect", rectBr);*/

                if (rect1.contains(contourCent)) {
                    spikeLocation = BVAutonomousBlueLeft.elementLocation.LEFT;
                    return spikeLocation;
                }
                if (rect2.contains(contourCent)) {
                    spikeLocation = BVAutonomousBlueLeft.elementLocation.MIDDLE;
                    return spikeLocation;
                }
            } else {
                telemetry.addData("Non-Element Contour Area", Imgproc.contourArea(contoursBlue.get(i)));
            }

            //Count trigger to put a time lock before method can declare contour as RIGHT
            if (i < contoursBlue.size()) {
                count += 1;
                telemetry.addData("Count", count);
                telemetry.update();
            }

            if (count >= 2500 && spikeLocation != elementLocation.LEFT && spikeLocation != elementLocation.MIDDLE) {
                spikeLocation = elementLocation.RIGHT;
                return spikeLocation;
            }
        }

        return spikeLocation;

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

            // Turn off RUN_TO_POSITION
            //backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}