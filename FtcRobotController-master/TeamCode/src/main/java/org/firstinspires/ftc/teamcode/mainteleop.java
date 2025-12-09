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
        public int isBallRear(){

        }
        currentState = RobotState.IDLE;



    }
}