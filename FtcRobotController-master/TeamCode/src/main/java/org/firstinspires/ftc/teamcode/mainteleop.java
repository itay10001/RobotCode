import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class mainteleop extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Dcmotor TransportMotor = hardwaremap.dcMotor.get("HM");
        Dcmotor ShootingMotor = hardwaremap.dcMotor.get("SM");
        Dcmotor IntakeMotor = hardwaremap.dcMotor.get("INTM");
        ColorSensor color1 = hardwareMap.get(ColorSensor.class, "color1");
        ColorSensor color2 = hardwareMap.get(ColorSensor.class, "color2");
        ColorSensor color3 = hardwareMap.get(ColorSensor.class, "color3");
        ColorSensor color4 = hardwareMap.get(ColorSensor.class, "color4");


        Public boolean isBallFront() {
            Return color1.alphe() > 500 || color2.alphe() > 500;
        }

        Public boolean isBallRear() {
            Return color3.alphe() > 500 || color4.alphe() > 500;
        }
        public enum RobotState {
            IDLE,
            HOLDING,
            INTAKE,
            SHOOTING
        }
        Public int getNumberOfBalls() {
            If(!isBallFront() && !isBallRear()) {
                Return 0;
            } else if (isBallFront() && !isBallRear()) {
                Return 1;
            } else if (isBallFront() && isBallRear()) {
                Return 3;
            }
        }

        switch (currentState) {
            case IDLE:
                IntakeMotor.setPower(0);
                TransportMotor.setPower(0)
                ShootingMotor.setPower(0);
                break;

            case INTAKE:
                IntakeMotor.setPower(1.0);
                TransportMotor.setPower(1.0)
                ShootingMotor.setPower(0);
                break;

            case HOLDING:
                IntakeMotor.setPower(0)
                TransportMotor.setPower(0)
                ShootingMotor.setPower(1.0);
            case SHOOTING:
                IntakeMotor.setPower(0);
                TransportMotor.setPower(0)
                ShootingMotor.setPower(1.0);
                break;
        }
        currentState = RobotState.IDLE;
        if (getNumberOfBalls == 0 && gamepad1.x = true):
        currentState = RobotState.INTAKE;
        else if (getNumberOfBalls == 3 && gamepad1.y = true):
        currentState = RobotState.HOLDING;
        else if (gamepad1.a = true):
        currentState = RobotState.SHOOTING;
        else if (getNumberOfBalls == 1):
        currentState = RobotState.IDLE;


    }
}