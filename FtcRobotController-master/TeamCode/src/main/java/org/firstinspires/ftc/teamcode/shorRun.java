package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "shortRun", group = "RED")
public class shorRun extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor FLM = hardwareMap.dcMotor.get("FLM");
        DcMotor BLM = hardwareMap.dcMotor.get("BLM");
        DcMotor FRM = hardwareMap.dcMotor.get("FRM");
        DcMotor BRM = hardwareMap.dcMotor.get("BRM");

        DcMotor m2 = hardwareMap.dcMotor.get("m2");
        DcMotor m3 = hardwareMap.dcMotor.get("m3");
        DcMotor m1 = hardwareMap.dcMotor.get("m1");

        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        if (isStopRequested()) return;

        double power = 0.4;


        long forward = 2000;   // how long to drive (ms)

        long shoot = 400;
        long shoot3 = 600;
        long shoothalt = 1000;

        m2.setPower(-0.68);
        m1.setPower(-1);

        sleep(3000);

        FLM.setPower(power);
        BLM.setPower(power);
        FRM.setPower(power);
        BRM.setPower(power);
        sleep(700);

        FLM.setPower(0);
        BLM.setPower(0);
        FRM.setPower(0);
        BRM.setPower(0);


        m3.setPower(-1);
        sleep(shoot);

        m3.setPower(0.1);
        sleep(shoothalt * 2);

        m3.setPower(-1);
        sleep(shoot);

        m3.setPower(0.1);
        sleep(shoothalt);

        m3.setPower(-1);
        sleep(shoot3);

        m3.setPower(0.1);
        sleep(shoothalt);


        FLM.setPower(-power);
        BLM.setPower(-power);
        FRM.setPower(power);
        BRM.setPower(power);
        sleep(700);

        FLM.setPower(0);
        BLM.setPower(0);
        FRM.setPower(0);
        BRM.setPower(0);

        FLM.setPower(-power);
        BLM.setPower(-power);
        FRM.setPower(-power);
        BRM.setPower(-power);
        sleep(3000);

        FLM.setPower(0);
        BLM.setPower(0);
        FRM.setPower(0);
        BRM.setPower(0);



        // STOP


        sleep(200);
    }
}
