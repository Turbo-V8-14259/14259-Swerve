package org.firstinspires.ftc.teamcode;

//imports
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.checkerframework.checker.units.qual.C;

public class CasterSwerve{


    Module moduleFR;
    Module moduleFL;
    Module moduleBR;
    Module moduleBL;






    public CasterSwerve(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4,
                        DcMotor motor5, DcMotor motor6, DcMotor motor7, DcMotor motor8,
                        RevColorSensorV3 homeSensor1, RevColorSensorV3 homeSensor2, RevColorSensorV3 homeSensor3, RevColorSensorV3 homeSensor4) {

        //setup modules
        moduleFR = new Module(motor1, motor2, homeSensor1, Math.toRadians(135), Math.toRadians(-17), 0.1);
        moduleFL = new Module(motor3, motor4, homeSensor2, Math.toRadians(-135), Math.toRadians(-17), 0.1);
        moduleBR = new Module(motor5, motor6, homeSensor3, Math.toRadians(-45), Math.toRadians(163), -0.1);
        moduleBL = new Module(motor7, motor8, homeSensor4, Math.toRadians(45), Math.toRadians(163), -0.1);
    }







    public void initCasterSwerve() {
        //home the modules
//        homeModules();
    }




    public void drive(double straightPower, double strafePower, double spinPower) {

        double xPower = strafePower;
        double yPower = straightPower;
        double turnPower = spinPower;


        // calculate raw motor powers for each module
        Module.MotorPowers modulePowersFR = moduleFR.calculateRawMotorPowers(xPower, yPower, turnPower);
        Module.MotorPowers modulePowersFL = moduleFL.calculateRawMotorPowers(xPower, yPower, turnPower);
        Module.MotorPowers modulePowersBR = moduleBR.calculateRawMotorPowers(xPower, yPower, turnPower);
        Module.MotorPowers modulePowersBL = moduleBL.calculateRawMotorPowers(xPower, yPower, turnPower);


        // run module calculations, determine motor power scale
        double motorPowerScale = 1/maxMotorMag(modulePowersFR, modulePowersFL, modulePowersBR, modulePowersBL);

        // if there's no need to scale then don't scale
        if(motorPowerScale > 1) { motorPowerScale = 1.0; }

        //apply motor powers in modules with scale factor
//        moduleFR.applyPowers(modulePowersFR, motorPowerScale);
        moduleFL.applyPowers(modulePowersFL, motorPowerScale);
        moduleBR.applyPowers(modulePowersBR, motorPowerScale);
        moduleBL.applyPowers(modulePowersBL, motorPowerScale);



        //override to home the modules in teleop, will complete homing sequence even if you release the button
//        boolean homeModules = false; // put in button
//        if(homeModules) {
//            moduleFR.setHomed(false);
//            moduleFL.setHomed(false);
//            moduleBR.setHomed(false);
//            moduleBL.setHomed(false);
//
//            // halt the loop to home them lol
//            while(!homeModules()) { }
//        }
        boolean homeModules = true; // put in button
        if(homeModules) {
            moduleFR.setHomed(true);
            moduleFL.setHomed(true);
            moduleBR.setHomed(true);
            moduleBL.setHomed(true);

            // halt the loop to home them lol
            while(!homeModules()) { }
        }


    }

    //returns the max motor magnitude from the return objects
    public double maxMotorMag(Module.MotorPowers module1Powers, Module.MotorPowers module2Powers,
                              Module.MotorPowers module3Powers, Module.MotorPowers module4Powers) {

        double max = module1Powers.getMaxMag();
        if(module2Powers.getMaxMag() > max) { max = module2Powers.getMaxMag(); }
        if(module3Powers.getMaxMag() > max) { max = module3Powers.getMaxMag(); }
        if(module4Powers.getMaxMag() > max) { max = module4Powers.getMaxMag(); }

        return max;
    }



    //homes all the modules and turns true when done (broken)
    public boolean homeModules() {

        moduleFR.homeModule();
        moduleFL.homeModule();
        moduleBR.homeModule();
        moduleBL.homeModule();

        return moduleFR.getHomed() && moduleFL.getHomed() && moduleBR.getHomed() && moduleBL.getHomed();
    }

}


