package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.BVColorProcessor.blueContourArea;
import static org.firstinspires.ftc.teamcode.BVColorProcessor.blueProcessor;
import static org.firstinspires.ftc.teamcode.BVColorProcessor.contoursBlue;
import static org.firstinspires.ftc.teamcode.BVColorProcessor.merge;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

public class BVColorAutoBlue extends LinearOpMode {

    final Scalar RED = new Scalar(255, 0, 0);

    OpenCvWebcam webcam;

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
                    Imgproc.drawContours(merge, contoursBlue, i, RED, 2, Imgproc.LINE_8);
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
            }
        }
    }
