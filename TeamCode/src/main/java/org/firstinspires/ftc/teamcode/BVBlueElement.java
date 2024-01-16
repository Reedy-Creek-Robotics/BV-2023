package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
public class BVBlueElement extends LinearOpMode {

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
        RIGHT
    }

    BVBlueElement.elementLocation spikeLocation = BVBlueElement.elementLocation.RIGHT;

    //--------------------------------------------------------

    public void runOpMode() throws InterruptedException {

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

            BVBlueElement.this.contoursBlue = contours;

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

    while (opModeInInit()) {
        telemetry.addLine("KEEP THE ROBOT NEAR THE LEFT SIDE OF THE SQUARE");
    }

    waitForStart();

    while (opModeIsActive()) {

        List<MatOfPoint> contoursBlue = BVBlueElement.this.contoursBlue;

        webcam.setPipeline(blueProcessor);

        telemetry.addLine("KEEP THE ROBOT NEAR THE LEFT SIDE OF THE SQUARE");
        telemetry.addLine("Detecting BLUE Contours");
        telemetry.addData("Webcam pipeline activity", webcam.getPipelineTimeMs());
        telemetry.addData("Contours Detected", contoursBlue.size());
        telemetry.addData("Contour Minimum Vision", contourMinimum);

        if (gamepad1.right_stick_y > 0.3) {
            contourMinimum -= 1;
        } if (gamepad1.right_stick_y < -0.3) {
            contourMinimum += 1;
        }

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
                    spikeLocation = BVBlueElement.elementLocation.LEFT;
                } else if (rect2.contains(contourCent)) {
                    spikeLocation = BVBlueElement.elementLocation.MIDDLE;
                }
            } else {
                telemetry.addData("Non-Element Contour Area", Imgproc.contourArea(contoursBlue.get(i)));
            }
        }

        if (spikeLocation == BVBlueElement.elementLocation.RIGHT) {
            telemetry.addLine("Element on NO Rectangle / RIGHT spike");
        } else if (spikeLocation == BVBlueElement.elementLocation.LEFT) {
            telemetry.addLine("Element on LEFT Rectangle / LEFT spike");
        } else if (spikeLocation == BVBlueElement.elementLocation.MIDDLE) {
            telemetry.addLine("Element on RIGHT Rectangle / MIDDLE spike");
        }

        telemetry.update();

        }
    }
}
