package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.TeleopTest.State.MANUAL;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleopTest", group = "TeleOp")
public class TeleopTest extends LinearOpMode {
    enum State {
        MANUAL,
        AUTOMATIC;
    }

    @Override
    public void runOpMode() throws InterruptedException {


        DcMotorEx Shooter1 = hardwareMap.get(DcMotorEx.class, "0"); // port 1
        DcMotorEx Shooter2 = hardwareMap.get(DcMotorEx.class, "1"); //port 0
        DcMotor FLM = hardwareMap.dcMotor.get("FLM");
        DcMotor BLM = hardwareMap.dcMotor.get("BLM");
        DcMotor FRM = hardwareMap.dcMotor.get("FRM");
        DcMotor BRM = hardwareMap.dcMotor.get("BRM");
        Servo hood = hardwareMap.servo.get("hood");
        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);
        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        Shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        Shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        DcMotor intake = hardwareMap.get(DcMotor.class, "2");
        DcMotor feeder = hardwareMap.get(DcMotor.class, "3");
        IMU imu = hardwareMap.get(IMU.class, "imu");
// Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);


        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();

        waitForStart();

        double hoodIncrement = 0.005;
        double hoodPosition = 0;
        double shooterVelocity =90;
        while (opModeIsActive()) {

            double ty = 0;
            double tx = 0;
            double w = 0;
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    ty = result.getTy();
                    tx = result.getTx();
                    w = -11.1*ty + 1688 * 1.036
                    ;
                }
            }


            Shooter1.setVelocity(w * 0.955);
            Shooter2.setVelocity(w * 0.955);

            if (gamepad1.left_trigger > 0.5 || gamepad1.left_bumper) {
                intake.setPower(-0.8);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.right_trigger > 0.5 ) {
                feeder.setPower(-0.8);
            } else {
                feeder.setPower(0);
            }
            if (gamepad1.right_bumper) {
                Shooter1.setVelocity(1500);
                Shooter2.setVelocity(1500);
            }

            double y = 0; // Remember, Y stick value is reversed
            double x = 0;
            double rx = 0;

            if (gamepad1.left_bumper ) {
                rx = tx * -0.02;
            } else {

                y = -gamepad1.left_stick_y ; // Remember, Y stick value is reversed
                x = gamepad1.left_stick_x ;
                rx = -gamepad1.right_stick_x;
            }
            if(gamepad1.a){
                Shooter1.setVelocity(-1800);
                Shooter2.setVelocity(-1800);
                intake.setPower(1);
                feeder.setPower(1);
            }

            if (gamepad1.dpad_up) {
                // Get current position and add the increment, ensuring it doesn't exceed 1.0
                hoodPosition = Math.min(0.45, hood.getPosition() + hoodIncrement);
            } else if (gamepad1.dpad_down) {
                //  Get current position and subtract the increment, ensuring it doesn't go below 0.0
                hoodPosition = Math.max(0.0, hood.getPosition() - hoodIncrement);

            }
            if (gamepad1.touchpad){
                State currentstate = MANUAL;
                double position = 0;

                double v = -0.0144* ty + 0.746;
                position = v;


                if (v > 0.45) {
                    position = 0.45;
                } else if (v < 0) {
                    position = 0;
                }

                hood.setPosition(position);//position
            } else {
                State currentstate = MANUAL;

            }



            if (gamepad1.options) {
                imu.resetYaw();
            }

            //0
            //0.44

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

            double v = -0.0144* ty + 0.746;
            FLM.setPower(frontLeftPower * 2);
            BLM.setPower(backLeftPower * 2);
            FRM.setPower(frontRightPower * 2);
            BRM.setPower(backRightPower * 2);

            telemetry.addData("Hood Position", hood.getPosition());
            telemetry.addData("Velo", Shooter1.getVelocity());
            telemetry.addData("v = ",v);
            telemetry.update();

        }
    }



    }

