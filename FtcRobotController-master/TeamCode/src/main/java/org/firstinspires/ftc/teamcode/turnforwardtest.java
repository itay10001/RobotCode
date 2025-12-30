package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "turnforwardtest", group = "Test")
public class turnforwardtest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor FLM = hardwareMap.dcMotor.get("FLM");
        DcMotor BLM = hardwareMap.dcMotor.get("BLM");
        DcMotor BRM = hardwareMap.dcMotor.get("BRM");
        DcMotor FRM = hardwareMap.dcMotor.get("FRM");
        DcMotor m2 = hardwareMap.dcMotor.get("m2");

        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        m2.setPower (-0.5);
        if (isStopRequested()) return;

        double power = 0.4;

        long forward = 1500;
        long turn = 180;

        FLM.setPower(-0.50);
        BLM.setPower(-0.50);
        FRM.setPower(-0.75);
        BRM.setPower(-0.75);
        sleep(forward);








    }
}


