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
public class BVColorAutoRed extends LinearOpMode {

    //HSV Red
    final Scalar LOW_RED1 = new Scalar(248, 100, 100);
    final Scalar HIGH_RED1 = new Scalar(0, 255, 255);

    final Scalar TEST_HIGH = new Scalar(255, 255, 255);
    final Scalar TEST_LOW = new Scalar(0, 0, 0);

    final Scalar LOW_RED2 = new Scalar(0, 100, 100);
    final Scalar HIGH_RED2 = new Scalar(5, 255, 255);

    final Scalar BLUE = new Scalar(0, 255, 0);

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

    Mat hierarchy = new Mat();

    Mat kernel = Mat.ones(7, 7, CvType.CV_8UC1);

    double redContourArea;

    List<MatOfPoint> contoursRed = new ArrayList<>();

    Mat merge = new Mat();

    //--------------------------------------------------------

    public void runOpMode() throws InterruptedException {

        OpenCvWebcam webcam;

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

                //Merges the two mats in a bitwise_or fashion
                Core.bitwise_or(morph1, morph2, merge);

                //Creates a list (array) of contours based on the now morphed image
                List<MatOfPoint> contours = new ArrayList<>();

                Imgproc.findContours(merge, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

                BVColorAutoRed.this.contoursRed = contours;

                for (int i = 0; i < contours.size(); i++) {
                    BVColorAutoRed.this.redContourArea = Imgproc.contourArea(contours.get(i));
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
                telemetry.addLine("INITIALIZATION SUCCESSFUL");
                telemetry.update();

                webcam.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("ERROR UPON INITIALIZATION:", errorCode);
                telemetry.update();
            }
        });

        while (opModeInInit()) {

            List<MatOfPoint> contoursRed = BVColorAutoRed.this.contoursRed;

            for (int i = 0; i < contoursRed.size(); i++) {
                Imgproc.drawContours(merge, contoursRed, i, BLUE, 2, Imgproc.LINE_8);

                telemetry.addLine("Drawing RED Contours...");
                telemetry.update();
            }
        }

        waitForStart();

        telemetry.clearAll();

        while (opModeIsActive()) {

            List<MatOfPoint> contoursRed = BVColorAutoRed.this.contoursRed;
            double redContourArea = BVColorAutoRed.this.redContourArea;

            webcam.setPipeline(redProcessor);

            telemetry.addLine("Detecting RED Contours");

            telemetry.addData("Webcam pipeline activity", webcam.getPipelineTimeMs());

            telemetry.addData("Contours Detected", contoursRed.size());

            for (int i = 0; i < contoursRed.size(); i++) {
                telemetry.addData("Current redContourArea", redContourArea);
                telemetry.update();
                if (redContourArea > 1000) {
                    telemetry.addData("Element Detected! Area of Element:", redContourArea);
                }
            }

            telemetry.update();

        }
    }
}
