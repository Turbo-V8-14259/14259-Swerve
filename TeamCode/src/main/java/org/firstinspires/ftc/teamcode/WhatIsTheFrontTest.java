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
        FRL = hardwareMap.get(DcMotor.class, "FRL");
        FRR = hardwareMap.get(DcMotor.class, "FRR");
        BLR = hardwareMap.get(DcMotor.class, "BLR");
        BLL= hardwareMap.get(DcMotor.class, "BLL");
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
            c.drive((gamepad1.left_stick_y + gamepad2.right_stick_y) * .5, (gamepad1.left_stick_x + gamepad2.right_stick_x) * .5, (-gamepad1.right_stick_x + -gamepad2.left_stick_x) * .5);

            telemetry.addData("front left module heading ", Math.toDegrees(c.getHeadingFL()));
            telemetry.addData("back left module heading ", Math.toDegrees(c.getHeadingBL()));
            telemetry.addData("front right module heading ", Math.toDegrees(c.getHeadingFR()));
            telemetry.addData("back right module heading ", Math.toDegrees(c.getHeadingBR()));
            telemetry.update();

        }
    }
}
