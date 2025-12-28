package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "breakTEST", group = "Test")
public class braketest extends LinearOpMode {
    // Drivetrain Motors
    private DcMotorEx FLM, FRM, BLM, BRM;

    // Mechanism Motors
    // m1: Intake, m2: Transfer, m3: Shooter
    private DcMotorEx m1, m2, m3;

    private IMU imu;

    // Active Braking Constants
    double kBrake = 0.00035;
    double maxBrakePower = 0.35;
    double deadband = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Drivetrain
        FLM = hardwareMap.get(DcMotorEx.class, "FLM");
        FRM = hardwareMap.get(DcMotorEx.class, "FRM");
        BLM = hardwareMap.get(DcMotorEx.class, "BLM");
        BRM = hardwareMap.get(DcMotorEx.class, "BRM");

        // Initialize Mechanisms (m1=Intake, m2=Transfer, m3=Shooter)
        m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");

        // Set Drivetrain to Brake mode
        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Directions (FR and BR reversed for Mecanum)
        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU Setup for Field Centric
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            // --- INPUTS ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            // --- MECHANISM LOGIC ---

            // m1 (Intake) Logic:
            // Hold R-Trigger on G1 to intake fast and slow down drive for precision
            if (gamepad1.right_trigger > 0.5) {
                y *= 0.5;
                x *= 0.5;
                rx *= 0.5;
                m1.setPower(-1.0); // Full intake
            } else {
                m1.setPower(-0.4); // Idle intake/hold
            }

            // m2 (Transfer) & m3 (Shooter) Logic:
            // Holding R-Trigger on G2 primes the shooter (m3) and runs transfer (m2)
            double m3Power = 0;
            if (gamepad2.right_trigger > 0.5) {
                m3Power = -1.0;     // Run Shooter
                m2.setPower(-0.78); // Run Transfer
            } else {
                m2.setPower(-2); // Idle/Holding Transfer
            }

            // Holding L-Trigger on G2 just runs the shooter
            if (gamepad2.left_trigger > 0.5) {
                m3Power = -1.0;
            }
            m3.setPower(m3Power);

            // --- DRIVE LOGIC ---

            // Reset Heading
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Field Centric Rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1; // Multiplier to fix mecanum strafe friction

            // Check if sticks are released
            boolean driverWantsStop = Math.abs(y) < deadband && Math.abs(x) < deadband && Math.abs(rx) < deadband;

            if (!driverWantsStop) {
                // Normal Movement
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                FLM.setPower((rotY + rotX + rx) / denominator);
                BLM.setPower((rotY - rotX + rx) / denominator);
                FRM.setPower((rotY - rotX - rx) / denominator);
                BRM.setPower((rotY + rotX - rx) / denominator);
            } else {
                // Active Braking Algorithm
                brakeMotor(FLM);
                brakeMotor(FRM);
                brakeMotor(BLM);
                brakeMotor(BRM);
            }

            // --- TELEMETRY ---
            telemetry.addData("Status", "excuse me coming through");
            telemetry.addData("Heading (rad)", -botHeading);
            telemetry.addData("Shooter Power", m3Power);
            telemetry.update();
        }
    }

    /**
     * Active Braking: Uses motor velocity to calculate a counter-power
     * to stop the robot faster than passive braking.
     */
    private void brakeMotor(DcMotorEx m) {
        double vel = m.getVelocity();
        double p = -kBrake * vel;
        // Clamp the power using our polite clip method
        p = clip(p, -maxBrakePower, maxBrakePower);
        m.setPower(p);
    }

    /**
     * The polite clipping method
     * @param v Current value
     * @param hey The Low limit
     * @param hi The High limit
     */
    private double clip(double v, double hey, double hi) {
        return Math.max(hey, Math.min(hi, v));
    }
}
