package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
import java.util.List;

@Autonomous
public class BVColorAutoBlue extends LinearOpMode {

    final Scalar RED = new Scalar(255, 0, 0);

    //HSV Blue
    final Scalar LOW_BLUE = new Scalar(100, 100, 100);
    final Scalar HIGH_BLUE = new Scalar(130, 255, 255);

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

            for (int i = 0; i < contoursBlue.size(); i++) {
                blueContourArea = Imgproc.contourArea(contoursBlue.get(i));
            }

            //Returns input to webcam
            return input;
        }
    };

        @Override
        public void runOpMode() throws InterruptedException {

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
                    webcam.setPipeline(blueProcessor);
                }

                @Override
                public void onError(int errorCode) {

                }
            });

            while (opModeInInit()) {
                for (int i = 0; i < contoursBlue.size(); i++) {
                    Imgproc.drawContours(morph, contoursBlue, i, RED, 2, Imgproc.LINE_8);
                }
            }

            waitForStart();

            while (opModeIsActive()) {

                telemetry.addLine("Detecting BLUE Contours");

                webcam.setPipeline(blueProcessor);

                telemetry.addData("Contours Detected", contoursBlue.size());

                for (int i = 0; i < contoursBlue.size(); i++) {
                    telemetry.addData("Contour Points: ", contoursBlue);

                    if (blueContourArea > 1000) {
                        telemetry.addData("Element Detected! Area of Element:", blueContourArea);
                    }
                }

                telemetry.update();

            }
        }
    }
