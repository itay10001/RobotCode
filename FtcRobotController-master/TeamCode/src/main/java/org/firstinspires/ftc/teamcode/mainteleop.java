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

    private static final int BALL_THRESHOLD = 500;
    private static final double GATE_CLOSED = 0.0;
    private static final double GATE_OPEN   = 0.7;

    public enum RobotState {
        IDLE,
        INTAKE,
        HOLDING,
        SHOOTING
    }

    private RobotState currentState = RobotState.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        transportMotor = hardwareMap.get(DcMotor.class, "HM");
        shootingMotor  = hardwareMap.get(DcMotor.class, "SM");
        intakeMotor    = hardwareMap.get(DcMotor.class, "INTM");

        color1 = hardwareMap.get(ColorSensor.class, "color1");
        color2 = hardwareMap.get(ColorSensor.class, "color2");
        color3 = hardwareMap.get(ColorSensor.class, "color3");
        color4 = hardwareMap.get(ColorSensor.class, "color4");

        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setPosition(GATE_CLOSED);

        waitForStart();

        while (opModeIsActive()) {

            int ballCount = getNumberOfBalls();
            boolean hasBalls  = ballCount > 0;
            boolean full      = ballCount == 3;
            boolean lessThan3 = ballCount < 3;

            boolean button1 = gamepad1.x;
            boolean button2 = gamepad1.a;
            boolean button3 = gamepad1.y;

            if (hasBalls && button2) {
                currentState = RobotState.SHOOTING;
            }
            else if (lessThan3 && button1) {
                currentState = RobotState.INTAKE;
            }
            else if ((full && currentState == RobotState.INTAKE) ||
                     (button3 && hasBalls)) {
                currentState = RobotState.HOLDING;
            }
            else if ((!hasBalls && currentState == RobotState.SHOOTING) ||
                     (button3 && !hasBalls)) {
                currentState = RobotState.IDLE;
            }

            setStateOutputs(currentState);

            telemetry.addData("State", currentState);
            telemetry.addData("Balls", ballCount);


            telemetry.addData("color1 alpha", color1.alpha());
            telemetry.addData("color2 alpha", color2.alpha());
            telemetry.addData("color3 alpha", color3.alpha());
            telemetry.addData("color4 alpha", color4.alpha());

            telemetry.update();
        }
    }

    private boolean isBallFront() {
        return (color1.alpha() > BALL_THRESHOLD) ||
               (color2.alpha() > BALL_THRESHOLD);
    }

    private boolean isBallRear() {
        return (color3.alpha() > BALL_THRESHOLD) ||
               (color4.alpha() > BALL_THRESHOLD);
    }

    private int getNumberOfBalls() {
        boolean front = isBallFront();
        boolean rear  = isBallRear();

        if (!front && !rear) {
            return 0;
        } else if (front && !rear) {
            return 1;
        } else if (front && rear) {
            return 3;
        } else if (!front && rear) {
            return 3;
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
                intakeMotor.setPower(1.0);
                transportMotor.setPower(1.0);
                shootingMotor.setPower(0.0);
                break;

            case HOLDING:
                gateServo.setPosition(GATE_CLOSED);
                intakeMotor.setPower(0.0);
                transportMotor.setPower(0.0);
                shootingMotor.setPower(1.0);
                break;

            case SHOOTING:
                gateServo.setPosition(GATE_OPEN);
                intakeMotor.setPower(0.0);
                transportMotor.setPower(1.0);
                shootingMotor.setPower(1.0);
                break;
        }
    }
}