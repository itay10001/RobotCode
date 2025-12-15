package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "test")
public class test extends LinearOpMode {
    private ColorSensor color1;
    private ColorSensor color2;
    private ColorSensor color3;
    private ColorSensor color4;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor m1 = hardwareMap.dcMotor.get("m1");
        DcMotor m2 = hardwareMap.dcMotor.get("m2");
        DcMotor m3 = hardwareMap.dcMotor.get("m3");
        color1 = hardwareMap.get(ColorSensor.class, "color1");
        color2 = hardwareMap.get(ColorSensor.class, "color2");
        color3 = hardwareMap.get(ColorSensor.class, "color3");
        color4 = hardwareMap.get(ColorSensor.class, "color4");

        waitForStart();

        // Main control loop
        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y; // usually inverted so up is positive

            m1.setPower(power);
            m2.setPower(-0.70);
            m3.setPower(power);
            telemetry.addData("color1 alpha", color1.alpha());
            telemetry.addData("color2 alpha", color2.alpha());
            telemetry.addData("color3 alpha", color3.alpha());
            telemetry.addData("color4 alpha", color4.alpha());
            telemetry.addData("Power", power);
            telemetry.update();
        }
        // (Optional) stop motors explicitly when opmode ends
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
    }
}
