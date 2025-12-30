package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "assaftest", group = "Test")
public class assaftest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor FLM = hardwareMap.dcMotor.get("FLM");
        DcMotor BLM = hardwareMap.dcMotor.get("BLM");
        DcMotor BRM = hardwareMap.dcMotor.get("BRM");
        DcMotor FRM = hardwareMap.dcMotor.get("FRM");


        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        if (isStopRequested()) return;

        double power = 0.4;

        long forward = 1000;

        FLM.setPower(-0.75);
        BLM.setPower(-0.75);
        FRM.setPower(-0.75);
        BRM.setPower(-0.75);
        sleep(forward);

    }
}


