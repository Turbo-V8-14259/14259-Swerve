package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SwerveDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //1 is inner
        //2 is outer
        DcMotor moduleFRinner;
        DcMotor moduleFRouter;
        DcMotor moduleFLinner;
        DcMotor moduleFLouter;
        DcMotor moduleBRinner;
        DcMotor moduleBRouter;
        DcMotor moduleBLinner;
        DcMotor moduleBLouter;

        moduleFRinner= hardwareMap.get(DcMotor.class, "FRI");
        moduleFRouter= hardwareMap.get(DcMotor.class, "FRO");
        moduleFLinner= hardwareMap.get(DcMotor.class, "FLI");
        moduleFLouter= hardwareMap.get(DcMotor.class, "FLO");
        moduleBRinner= hardwareMap.get(DcMotor.class, "BRI");
        moduleBRouter= hardwareMap.get(DcMotor.class, "BRO");
        moduleBLinner= hardwareMap.get(DcMotor.class, "BLI");
        moduleBLouter= hardwareMap.get(DcMotor.class, "BLO");


        CasterSwerve c = new CasterSwerve(moduleFRinner, moduleFRouter, moduleFLinner, moduleFLouter, moduleBRinner, moduleBRouter, moduleBLinner, moduleBLouter);

        while(opModeInInit()) {
//            c.initCasterSwerve();
//            moduleFLinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            moduleFLouter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
        while(opModeIsActive()) {
            c.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//            telemetry.addData("heading of the Front Left Module", Math.toDegrees(c.moduleFL.calculateHeading(moduleFLinner.getCurrentPosition(), moduleFLouter.getCurrentPosition())));
            telemetry.addData("inner",-moduleFLinner.getCurrentPosition());
            telemetry.addData("outer",-moduleFLouter.getCurrentPosition()); //FL
            telemetry.update();
        }
    }
}
