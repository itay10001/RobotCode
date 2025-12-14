package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "test")
public class test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor m1 = hardwareMap.dcMotor.get("m1");
        DcMotor m2 = hardwareMap.dcMotor.get("m2");
        DcMotor m3 = hardwareMap.dcMotor.get("m3");

        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y; // usually inverted so up is positive

            m1.setPower(gamepad1.left_stick_y * 1);
            m2.setPower(gamepad1.right_stick_y * 1);
            m3.setPower(gamepad1.left_stick_y * 1);

            telemetry.addData("Power", power);
            telemetry.update();
        }

        // (Optional) stop motors explicitly when opmode ends
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
    }
}
