package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class LLtest extends LinearOpMode {

    double gatePosition = -0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FLM = hardwareMap.dcMotor.get("FLM");
        DcMotor BLM = hardwareMap.dcMotor.get("BLM");
        DcMotor FRM = hardwareMap.dcMotor.get("FRM");
        DcMotor BRM = hardwareMap.dcMotor.get("BRM");
        //Servo gate = hardwareMap.servo.get("gate");

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor m1 = hardwareMap.dcMotor.get("m1");
        DcMotor m2 = hardwareMap.dcMotor.get("m2");
        DcMotor m3 = hardwareMap.dcMotor.get("m3");

        IMU imu = hardwareMap.get(IMU.class, "imu");
// Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y ; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x ;
            double rx = -gamepad1.right_stick_x;





            double power = 0;

            if (gamepad1.right_trigger > 0.5){
                y = y *0.5;
                x = x*0.5;
                rx = rx*0.5;
                m1.setPower(-1);
            }   else {
                m1.setPower(-0.4);
            }

//            if (gamepad2.right_trigger > 0.5) {
//                power = -1;
//                m2.setPower(-0.78);
//            }   else{
//                m2.setPower(-0.60);
//            }

            if (gamepad1.left_trigger > 0.5) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    double tx = result.getTx(); // How far left or right the target is (degrees)
                    double ty = result.getTy(); // How far up or down the target is (degrees)
                    double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                    if (rx > -2.3){
                        rx = tx * -0.04 - 0.2;
                    } else{
                        rx = tx * -0.04 + 0.2;
                    }

                    telemetry.addData("Target X", tx);
                    telemetry.addData("Target Y", ty);
                    telemetry.addData("Target Area", ta);
                } else {
                    rx = 0;
                    telemetry.addData("Limelight", "No Targets");
                }
            }   else{
                m2.setPower(-0.78);
            }

            if (gamepad1.left_trigger > 0.5){
                power = -1;
            }

            m3.setPower(power);



            //galdo sybau
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;



            FLM.setPower(frontLeftPower);
            BLM.setPower(backLeftPower);
            FRM.setPower(frontRightPower);
            BRM.setPower(backRightPower);



            telemetry.addData("Angle:", -botHeading);
            telemetry.addData("gateposition: ", gatePosition);
            telemetry.update();

            System.out.println("hoo lee sheet");
        }
    }
}