package org.firstinspires.ftc.teamcode;

import static org.checkerframework.checker.nullness.Opt.get;

import android.util.Size;

import com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaLocalizerAccess;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp
public class BVAprilTagTest extends LinearOpMode  {

    //Our visionPortal / webCam
    VisionPortal webCam;

    //Our processor for the webCam
    AprilTagProcessor processor;

    //Exposure Control var init
    ExposureControl exposureController;

    @Override
    public void runOpMode() throws InterruptedException {

        //April tag processor init
        processor = new AprilTagProcessor.Builder()

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
        webCamBuilder.addProcessor(processor);


        // Build the Vision Portal, using the above settings.
        webCam = webCamBuilder.build();

        // Exposure control init
        // Program recieves an error for exposureController = null. Implement TFOD / Vuforia to fix this.
        //exposureController = exposureController.getClass();
        exposureController.setMode(ExposureControl.Mode.Manual);
        exposureController.setExposure(30, TimeUnit.MILLISECONDS);

        // Disable or re-enable the aprilTag processor at any time.
        //webCam.setProcessorEnabled(aprilTag, true);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // Telemetry for camera current settings
                telemetry.addData("Camera exposure:", exposureController.getExposure(TimeUnit.MILLISECONDS));

                // Telemetry for notice of april tag detection
                List<AprilTagDetection> currentDetections = processor.getDetections();
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
                }   // end for() loop

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    webCam.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    webCam.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        webCam.close();

    }   // end method runOpMode()
}
