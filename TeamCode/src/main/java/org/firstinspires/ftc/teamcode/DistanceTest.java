package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class DistanceTest extends LinearOpMode {
    DistanceSensor distanceSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor=hardwareMap.get(DistanceSensor.class, "distanceSensor");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

}
