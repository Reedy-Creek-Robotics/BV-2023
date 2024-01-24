package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class BVClawRotationTest extends LinearOpMode {

    Servo ClawRotation;

    public void runOpMode() {

        ClawRotation = hardwareMap.get(Servo.class, "ClawRotation");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                ClawRotation.setPosition(.445);
            }

            if (gamepad1.y) {
                ClawRotation.setPosition(.7);
            }
        }
    }
}
