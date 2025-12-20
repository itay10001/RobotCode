package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "longRED", group = "RED")
public class longRED extends LinearOpMode {

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

        long forward = 2500;   // how long to drive (ms)
        long turn = 400;
        long incline = 1500;
        long forward2 = 475;
        long halt = 500;
        long shoot = 400;
        long shoot3 = 600;
        long shoothalt = 1000;

        m2.setPower(-0.64);
        m1.setPower(-1);

        FLM.setPower(-power);
        BLM.setPower(-power);
        FRM.setPower(-power);
        BRM.setPower(-power);
        sleep(forward);

        FRM.setPower(power);
        FLM.setPower(-power);
        BRM.setPower(power);
        BLM.setPower(-power);
        sleep(turn);

        FRM.setPower(-power);
        FLM.setPower(0);
        BRM.setPower(0);
        BLM.setPower(-power);
        sleep(incline);

        FLM.setPower(-power);
        BLM.setPower(-power);
        FRM.setPower(-power);
        BRM.setPower(-power);
        sleep(forward2);

        FLM.setPower(0);
        BLM.setPower(0);
        FRM.setPower(0);
        BRM.setPower(0);
        sleep(halt);

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


        // STOP


        sleep(200);
    }
}
