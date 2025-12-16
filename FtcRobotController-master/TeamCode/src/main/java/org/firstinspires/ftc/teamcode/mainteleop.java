package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "mainteleop")
public class mainteleop extends LinearOpMode {

    private DcMotor transportMotor;
    private DcMotor shootingMotor;
    private DcMotor intakeMotor;

    private ColorSensor color1;
    private ColorSensor color2;
    private ColorSensor color3;
    private ColorSensor color4;

    private Servo gateServo;

    // === CONSTANTS ===

    private static final double GATE_CLOSED = 0.0;
    private static final double GATE_OPEN   = 0.7;

    private static final double INTAKE_POWER           = -1.0;
    private static final double TRANSPORT_POWER_INTAKE = -1.0;
    private static final double TRANSPORT_POWER_SHOOT  = -1.0;

    public enum RobotState {
        IDLE,
        INTAKE,
        HOLDING,
        SHOOTING
    }

    private RobotState currentState = RobotState.IDLE;

    // === FAILSAFE STATES ===
    // B → pretend 1 ball (when sensors are dead)
    private boolean failsafeOneEnabled = false;
    private boolean lastB = false;

    // Y → pretend 3 balls (when sensors are dead)
    private boolean failsafeThreeEnabled = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware init
        transportMotor = hardwareMap.get(DcMotor.class, "holdingMotor");
        shootingMotor  = hardwareMap.get(DcMotor.class, "SM");
        intakeMotor    = hardwareMap.get(DcMotor.class, "INTM");

        color1 = hardwareMap.get(ColorSensor.class, "color1");
        color2 = hardwareMap.get(ColorSensor.class, "color2");
        color3 = hardwareMap.get(ColorSensor.class, "color3");
        color4 = hardwareMap.get(ColorSensor.class, "color4");

        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setPosition(GATE_CLOSED);

        // Optional: set directions (tune these for your robot)
        transportMotor.setDirection(DcMotor.Direction.FORWARD);
        shootingMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Optional: brake when power = 0
        transportMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            // Read buttons
            boolean buttonIntake = gamepad1.x;           // intake
            boolean buttonShoot  = gamepad1.a;           // shoot
            boolean buttonY      = gamepad1.y;           // used in state + 3-ball failsafe toggle
            boolean buttonB      = gamepad1.b;           // 1-ball failsafe toggle
            boolean manualGateOpen = gamepad1.right_bumper; // panic gate override

            // === FAILSAFE TOGGLES ===
            // B toggles "pretend 1 ball" (mutually exclusive with 3-ball)
            if (buttonB && !lastB) {
                failsafeOneEnabled = !failsafeOneEnabled;
                if (failsafeOneEnabled) {
                    failsafeThreeEnabled = false;
                }
            }
            lastB = buttonB;

            // Y toggles "pretend 3 balls" (mutually exclusive with 1-ball)
            if (buttonY && !lastY) {
                failsafeThreeEnabled = !failsafeThreeEnabled;
                if (failsafeThreeEnabled) {
                    failsafeOneEnabled = false;
                }
            }
            lastY = buttonY;

            int ballCount = getNumberOfBalls();
            boolean hasBalls  = ballCount > 0;
            boolean full      = ballCount == 3;
            boolean lessThan3 = ballCount < 3;

            // === STATE MACHINE ===
            if (hasBalls && buttonShoot) {
                currentState = RobotState.SHOOTING;
            }
            else if (lessThan3 && buttonIntake) {
                currentState = RobotState.INTAKE;
            }
            else if ((full && currentState == RobotState.INTAKE) ||
                    (buttonY && hasBalls)) {
                currentState = RobotState.HOLDING;
            }
            else if ((!hasBalls && currentState == RobotState.SHOOTING) ||
                    (buttonY && !hasBalls)) {
                currentState = RobotState.IDLE;
            }

            setStateOutputs(currentState);

            if (manualGateOpen) {
                gateServo.setPosition(GATE_OPEN);
            }

            // === TELEMETRY ===
            telemetry.addData("State", currentState);
            telemetry.addData("Balls", ballCount);

            telemetry.addData("Sensor health", sensorsOk() ? "OK" : "NO FEEDBACK");
            telemetry.addData("Failsafe 1-ball (B)", failsafeOneEnabled ? "ON" : "OFF");
            telemetry.addData("Failsafe 3-balls (Y)", failsafeThreeEnabled ? "ON" : "OFF");

            telemetry.addData("color1 alpha", color1.alpha());
            telemetry.addData("color2 alpha", color2.alpha());
            telemetry.addData("color3 alpha", color3.alpha());
            telemetry.addData("color4 alpha", color4.alpha());

            telemetry.update();
        }
    }

    private boolean sensorsOk() {
        int a1 = color1.alpha();
        int a2 = color2.alpha();
        int a3 = color3.alpha();
        int a4 = color4.alpha();


        return (a1 > 1000) || (a2 > 1000) || (a3 > 1000) || (a4 > 1000);
    }

    private boolean isBallFront() {
        return (color3.alpha() > 90) ||
                (color4.alpha() > 160);
    }

    private boolean isBallRear() {
        return (color1.alpha() > 310) ||
                (color2.alpha() > 180);
    }

    private int getNumberOfBalls() {

        boolean sensorsHealthy = sensorsOk();

//        // === ONLY use manual overrides when sensors have NO feedback ===
//        if (!sensorsHealthy) {
//            if (failsafeThreeEnabled) {
//                return 3;
//            }
//            if (failsafeOneEnabled) {
//                return 1;
//            }
//            // no sensor, no override → safest default
//            return 0;
//        }

        boolean front = isBallFront();
        boolean rear  = isBallRear();

        if (!front && !rear) {
            return 0;
        } else if (front && rear) {
            return 3;
        } else {
            // exactly one side sees a ball (front XOR rear) → treat as 1 ball
            return 1;
        }
    }

    private void setStateOutputs(RobotState state) {
        switch (state) {

            case IDLE:
                gateServo.setPosition(GATE_CLOSED);
                intakeMotor.setPower(0.0);
                transportMotor.setPower(0.0);
                shootingMotor.setPower(0.0);
                break;

            case INTAKE:
                gateServo.setPosition(GATE_CLOSED);
                intakeMotor.setPower(INTAKE_POWER);
                transportMotor.setPower(TRANSPORT_POWER_INTAKE);
                shootingMotor.setPower(0.0);
                break;

            case HOLDING:
                gateServo.setPosition(GATE_CLOSED);
                intakeMotor.setPower(0.0);
                transportMotor.setPower(0.0);
                shootingMotor.setPower(-0.5);
                break;

            case SHOOTING:
                gateServo.setPosition(GATE_OPEN);
                intakeMotor.setPower(0.0);
                transportMotor.setPower(TRANSPORT_POWER_SHOOT * 2);
                shootingMotor.setPower(-0.75);
                break;
        }
    }
}
