package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class WhatIsTheFrontTest extends LinearOpMode {
    DcMotor FLR;
    DcMotor FLL;
    DcMotor FRL;
    DcMotor FRR;
    DcMotor BLR;
    DcMotor BLL;
    DcMotor BRR;
    DcMotor BRL;

    @Override
    public void runOpMode() throws InterruptedException {
        FLR = hardwareMap.get(DcMotor.class, "FLR");
        FLL = hardwareMap.get(DcMotor.class, "FLL");
        //problem with the FL gotube part skipping
        FRL = hardwareMap.get(DcMotor.class, "FRL");
        FRR = hardwareMap.get(DcMotor.class, "FRR");
        BLR = hardwareMap.get(DcMotor.class, "BLR");
        BLL= hardwareMap.get(DcMotor.class, "BLL");
        //BL is good
        BRR= hardwareMap.get(DcMotor.class, "BRR");
        BRL= hardwareMap.get(DcMotor.class, "BRL");
        FLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FLL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BLL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        CasterSwerve c = new CasterSwerve(FRL, FRR, FLL, FLR, BRL, BRR, BLL, BLR);

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("back left module heading ", Math.toDegrees(c.getHeadingBL()));
            telemetry.addData("front right module heading ", Math.toDegrees(c.getHeadingFR()));
            telemetry.update();
//            BRL.setPower(gamepad1.left_trigger);
//            BRR.setPower(gamepad1.left_trigger);
        }
    }
}
