package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Timer;

@Autonomous(name = "assaftest", group = "Test")
public class assaftest extends LinearOpMode {
    private ElapsedTime autoTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx Shooter1 = hardwareMap.get(DcMotorEx.class, "0"); // port 1
        DcMotorEx Shooter2 = hardwareMap.get(DcMotorEx.class, "1"); //port 0
        DcMotor FLM = hardwareMap.dcMotor.get("FLM");
        DcMotor BLM = hardwareMap.dcMotor.get("BLM");
        DcMotor FRM = hardwareMap.dcMotor.get("FRM");
        DcMotor BRM = hardwareMap.dcMotor.get("BRM");
        DcMotor intake = hardwareMap.get(DcMotor.class, "2");
        DcMotor feeder = hardwareMap.get(DcMotor.class, "3");
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
        Shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        Shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        Shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));


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



            double power = 0.4;

            double backward = 0.25;
            FRM.setPower(power);
            FLM.setPower(power);
            BRM.setPower(power);
            BLM.setPower(power);
            sleep(1500);

            FRM.setPower(0);
            FLM.setPower(0);
            BRM.setPower(0);
            BLM.setPower(0);







                LLResult result = limelight.getLatestResult();
                double tx = 0;
                double ty = 0;
                double w = 0;
                double position = 0;
                if (result != null) {
                    if (result.isValid()) {
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("ty", result.getTy());
                        ty = result.getTy();
                        tx = result.getTx();
                        w = -11.1 * ty + 1688 * 1.036
                        ;
                    }
                }
                Shooter1.setVelocity(w);
                Shooter2.setVelocity(w);
                double v = -0.0144* ty + 0.746;
                position = v;

                if (v > 0.45) {
                    position = 0.45;
                } else if (v < 0) {
                    position = 0;
                }
                hood.setPosition(position);

                sleep(1000);
                intake.setPower(-1);

                sleep(1000);
                long turn = 400;
                FLM.setPower(power * 2.5);
                BLM.setPower(power * 2.5);
                FRM.setPower(-power * 2.5);
                BRM.setPower(-power * 2.5);
                sleep(turn);
                FRM.setPower(0);
                FLM.setPower(0);
                BRM.setPower(0);
                BLM.setPower(0);
                sleep(200);
                FLM.setPower(power);
                BRM.setPower(power);
                FRM.setPower(-power);
                BLM.setPower(-power);
                sleep(200);
        FLM.setPower(power);
        BRM.setPower(power);
        FRM.setPower(power);
        BLM.setPower(power);
        feeder.setPower(-1);
        intake.setPower(-0.68);
        sleep(2100);

        intake.setPower(0);
        FLM.setPower(-power);
        BRM.setPower(-power);
        FRM.setPower(-power);
        BLM.setPower(-power);
        sleep(1800);

        FLM.setPower(-power * 2.5);
        BLM.setPower(-power * 2.5);
        FRM.setPower(power * 2.5);
        BRM.setPower(power * 2.5);
        sleep(500);


        FRM.setPower(0);
        FLM.setPower(0);
        BRM.setPower(0);
        BLM.setPower(0);
        sleep(900);


        intake.setPower(-1);
        sleep(1000);





//                FLM.setPower(frontLeftPower * 2);
//                BLM.setPower(backLeftPower * 2);
//                FRM.setPower(frontRightPower * 2);
//                BRM.setPower(backRightPower * 2);

                telemetry.addData("Hood Position", hood.getPosition());
                telemetry.addData("Velo", Shooter1.getVelocity());
                telemetry.addData("v = ", v);
                telemetry.update();

            }
        }



//}

