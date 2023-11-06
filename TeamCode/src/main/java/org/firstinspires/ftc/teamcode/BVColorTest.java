package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp (name="BVCameraTest", group="Robot")
public class BVColorTest extends LinearOpMode {

    @Override
    public void runOpMode(){

        //Camera init
        VisionPortal visionPortal;
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Camera"));
        waitForStart();

        while (opModeIsActive()) {
            //If red sensed, telemetry message. If not, different telemetry message.

            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
                telemetry.addLine("Camera Disabled");
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
                telemetry.addLine("Camera Enabled");
            }
            telemetry.clearAll();
            telemetry.update();
            sleep(500);

        }

        visionPortal.close();

    }
}
