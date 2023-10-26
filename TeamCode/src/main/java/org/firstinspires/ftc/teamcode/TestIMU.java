package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



public class TestIMU extends LinearOpMode {
    BNO055IMU imu;

    DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
    DcMotor motorBackLeft = hardwareMap.dcMotor.get("BackLeft");
    DcMotor motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
    DcMotor motorBackRight = hardwareMap.dcMotor.get("BackRight");
    double y = -gamepad1.left_stick_y; // Remember, this is reversed!
    double rx = gamepad1.right_stick_x;

    private void initIMU() {
       /* this.imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));*/

        this.imu = hardwareMap.tryGet(BNO055IMU.class, "expansionImu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        telemetry.addLine("Imu is Initializing");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
            telemetry.clear();
            telemetry.addLine("Calibrating");
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
    }
}
