package org.firstinspires.ftc.teamcode;

import static com.google.blocks.ftcrobotcontroller.util.CurrentGame.TFOD_MODEL_ASSET;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodParameters.CurrentGame.LABELS;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


@TeleOp
public class BVWebCamTest extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "CenterStage.tflite";

    private static final String[] LABELS = {
            "Pixel",
    };

    // Method for aprilTag init: initAprilTagProcessor()
    // Method for tfod init: initTfodProcessor

    //Global vars for switch method
    boolean tfodSwitch;
    boolean tagSwitch;

    //Global VisionPortal for both processors
    VisionPortal webCam;

    //aprilTagProcessor init
    VisionProcessor tagProcessorSwitch;
    AprilTagProcessor tagProcessor;

    //tfodProcessor init
    VisionProcessor tfodProcessorSwitch;
    TfodProcessor tfodProcessor;


    @Override
    public void runOpMode() throws InterruptedException {

        //Tag + tfod inits
        initAprilTagProcessor();
        initTfodProcessor();

        //Active processor init
        //switchProcessor(true, true);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("Y to toggle tfod, A to toggle tag");

            //Toggles between tfod and tag
            if (gamepad1.y) {
                //switchProcessor(true, false);

                List<Recognition> currentRecognitions = tfodProcessor.getRecognitions();
                telemetry.addData("# Objects Detected", currentRecognitions.size());

                // Step through the list of recognitions and display info for each one.
                for (Recognition recognition : currentRecognitions) {
                    double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                    telemetry.addData(""," ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position", "%.0f / %.0f", x, y);
                    telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

                }
            } if (gamepad1.a) {
                //switchProcessor(false, true);

                List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
                telemetry.addData("# AprilTags Detected", currentDetections.size());

                // Step through the list of detections and display info for each one.
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    } else {
                        telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                    }
                }
            }

            telemetry.addData("IsTfodToggled:", tfodSwitch);
            telemetry.addData("IsTagToggled:", tagSwitch);

            telemetry.update();

        }

    }

    //Put methods for apriltag + tfod inits here
    private void initAprilTagProcessor() {

        //April tag processor init
        tagProcessor = new AprilTagProcessor.Builder()

                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                .build();

        // Vision Portal / Camera init
        VisionPortal.Builder webCamBuilder = new VisionPortal.Builder();
        webCamBuilder.setCamera(hardwareMap.get(WebcamName.class, "Camera"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        webCamBuilder.setCameraResolution(new Size(800, 600));

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        webCamBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        webCamBuilder.setAutoStopLiveView(true);

        // Set and enable the processor.
        webCamBuilder.addProcessor(tagProcessor);

        // Build the Vision Portal, using the above settings.
        webCam = webCamBuilder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //webCam.setProcessorEnabled(aprilTag, true);

    }

    private void initTfodProcessor() {

        // Create the TensorFlow processor by using a builder.
        tfodProcessor = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                .setModelInputSize(40)
                .setModelAspectRatio(16.0 / 12.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the webcam.
        builder.setCamera(hardwareMap.get(WebcamName.class, "Camera"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(800, 600));

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfodProcessor);

        // Build the Vision Portal, using the above settings.
        webCam = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /*private void switchProcessor(boolean tfodSwitch, boolean tagSwitch) {

        //Tfod logic
        if (tfodSwitch) {
            webCam.setProcessorEnabled(tfodProcessorSwitch, true);
        } else {
            webCam.setProcessorEnabled(tfodProcessorSwitch, false);
        }

        //Tag logic
        if (tagSwitch) {
            webCam.setProcessorEnabled(tagProcessorSwitch, true);
        } else {
            webCam.setProcessorEnabled(tagProcessorSwitch, false);
        }
    }*/
}