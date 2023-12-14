package org.firstinspires.ftc.teamcode;

import static org.opencv.core.CvType.CV_32F;
import static org.opencv.core.CvType.CV_32S;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp
public class BVColorTest extends LinearOpMode{

    //Global vars

    //RGB
    final Scalar BLUE = new Scalar(0, 0, 255);
    final Scalar RED = new Scalar(255, 0, 0);
    final Scalar PURPLE = new Scalar(255, 0, 255);
    final Scalar GREEN = new Scalar(0, 255, 0);

    //HSV Blue
    final Scalar LOW_BLUE = new Scalar(100, 100, 100);
    final Scalar HIGH_BLUE = new Scalar(130, 255, 255);

    //HSV Red [UNTESTED VALUES]
    final Scalar LOW_RED1 = new Scalar(248, 100, 100);
    final Scalar HIGH_RED1 = new Scalar(0, 255, 255);

    final Scalar LOW_RED2 = new Scalar(0, 100, 100);
    final Scalar HIGH_RED2 = new Scalar(5, 255, 255);

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
    //Output; stores the output for drawing contours if wanted
    Mat output = new Mat();
    //Blue contour area; stores a different contour area value depending on the iteration of the for loop it is on
    double blueContourArea;

    //--------------------------------------------------------

    //Red Processor Vars
    //Two vars of each since we are creating two comparisons then merging them.
    //Refer to blue processor comments for descriptions of mats.

    Mat hsvMat1 = new Mat();
    Mat hsvMat2 = new Mat();

    Mat inRangeMat1 = new Mat();
    Mat inRangeMat2 = new Mat();

    Mat morph1 = new Mat();
    Mat morph2 = new Mat();

    double redContourArea;

    List<MatOfPoint> contoursRed = new ArrayList<>();

    Mat merge = new Mat();



    //--------------------------------------------------------

    //Webcam initialization
    OpenCvWebcam webcam;

    //Pipeline initialization
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

            for (int i = 0; i < contoursBlue.size(); i++) {
                blueContourArea = Imgproc.contourArea(contoursBlue.get(i));

                if (blueContourArea > 1000) {
                    telemetry.addData("Element Detected! Area of Element:", blueContourArea);
                }
            }

            BVColorTest.this.contoursBlue = contours;

            //Returns input to webcam
            return input;
        }
    };

    OpenCvPipeline redProcessor = new OpenCvPipeline() {

        @Override
        public Mat processFrame(Mat input) {

            //Converts all color from RGB to HSV
            Imgproc.cvtColor(input, hsvMat1, Imgproc.COLOR_RGB2HSV);
            Imgproc.cvtColor(input, hsvMat2, Imgproc.COLOR_RGB2HSV);

            //Creates a bitmap based on if the color is within the two scalar values
            Core.inRange(hsvMat1, LOW_RED1, HIGH_RED1, inRangeMat1);
            Core.inRange(hsvMat2, LOW_RED2, HIGH_RED2, inRangeMat2);

            //Removes excess sound to create contours easily
            Imgproc.morphologyEx(inRangeMat1, morph1, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.morphologyEx(morph1, morph1, Imgproc.MORPH_OPEN, kernel);

            Imgproc.morphologyEx(inRangeMat2, morph2, Imgproc.MORPH_CLOSE, kernel);
            Imgproc.morphologyEx(morph2, morph2, Imgproc.MORPH_OPEN, kernel);

            Core.bitwise_or(morph1, morph2, merge);

            //Creates a list (array) of contours based on the now morphed image
            List<MatOfPoint> contours = new ArrayList<>();
            //Variable for the red contour area based on the contours

            Imgproc.findContours(merge, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            for (int i = 0; i < contoursRed.size(); i++) {
                redContourArea = Imgproc.contourArea(contoursRed.get(i));

                if (redContourArea > 1000) {
                    telemetry.addData("Element Detected! Area of Element:", redContourArea);
                }
            }

            BVColorTest.this.contoursRed = contours;

            //Returns input to webcam
            return input;
        }

    };

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Log.d("OPENCV", "WEBCAM STARTED STREAMING");
                webcam.startStreaming(800,600, OpenCvCameraRotation.UPRIGHT);
                webcam.setPipeline(redProcessor);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                telemetry.addLine("Detecting BLUE Contours");

                webcam.setPipeline(blueProcessor);

                List<MatOfPoint> contoursBlue = this.contoursBlue;

                telemetry.addData("Contours Detected", contoursBlue.size());

                for (int i = 0; i < contoursBlue.size(); i++) {
                    telemetry.addData("Contour Points: ", contoursBlue);
                }

            } if (gamepad1.b) {
                telemetry.addLine("Detecting RED Contours");

                webcam.setPipeline(redProcessor);

                List<MatOfPoint> contoursRed = this.contoursRed;

                telemetry.addData("Contours Detected", contoursRed.size());

                for (int i = 0; i < contoursBlue.size(); i++) {
                    telemetry.addData("Contour Points: ", contoursBlue);
                }
            }

            telemetry.update();

        }
    }
}