package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
        FRL.setDirection(DcMotorSimple.Direction.REVERSE);
        FRR.setDirection(DcMotorSimple.Direction.REVERSE);
        FLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FLL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BLL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        CasterSwerve c = new CasterSwerve(FRR, FRL, FLL, FLR, BRL, BRR, BLL, BLR);

        waitForStart();
        while(opModeIsActive()){
//            FRL.setPower(gamepad1.left_trigger);
//            FRR.setPower(gamepad1.left_trigger);
//            FLL.setPower(gamepad1.left_trigger);
//            FLR.setPower(gamepad1.left_trigger);
//            telemetry.addData("gp1 left trig", gamepad1.left_trigger);
            c.drive(gamepad1.left_stick_y * .5, gamepad1.left_stick_x * .5, gamepad1.right_stick_x * .5);
//            telemetry.addData("front right module heading ", Math.toDegrees(c.getHeadingFR()));

            telemetry.addData("front left module heading ", Math.toDegrees(c.getHeadingFL()));
            telemetry.addData("back left module heading ", Math.toDegrees(c.getHeadingBL()));
            telemetry.addData("front right module heading ", Math.toDegrees(c.getHeadingFR()));
            telemetry.addData("back right module heading ", Math.toDegrees(c.getHeadingBR()));

//            telemetry.addData("power 1 of FR", c.getBRP1());
//            telemetry.addData("power 2 of Fr", c.getBRP2());
            telemetry.update();
//            BRL.setPower(gamepad1.left_trigger);
//            BRR.setPower(gamepad1.left_trigger);
        }
    }
}
