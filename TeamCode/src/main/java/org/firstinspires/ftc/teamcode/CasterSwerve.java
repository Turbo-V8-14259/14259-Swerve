package org.firstinspires.ftc.teamcode;

//imports
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.checkerframework.checker.units.qual.C;

public class CasterSwerve {


    Module moduleFR;
    Module moduleFL;
    Module moduleBR;
    Module moduleBL;


    public CasterSwerve(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4,
                        DcMotor motor5, DcMotor motor6, DcMotor motor7, DcMotor motor8) {
        //setup modules
        moduleFR = new Module(motor1, motor2 , Math.toRadians(135), Math.toRadians(0));
        moduleFL = new Module(motor3, motor4, Math.toRadians(-135), Math.toRadians(0));
        moduleBR = new Module(motor5, motor6, Math.toRadians(-45), Math.toRadians(0));
        moduleBL = new Module(motor7, motor8, Math.toRadians(45), Math.toRadians(0));
    }


    public void drive(double straightPower, double strafePower, double spinPower) {
        double xPower = strafePower;
        double yPower = straightPower;
        double turnPower = spinPower;


        // calculate raw motor powers for each module
        Module.MotorPowers modulePowersFR = moduleFR.calculateRawMotorPowers(xPower, yPower, turnPower, -1, 1);
        Module.MotorPowers modulePowersFL = moduleFL.calculateRawMotorPowers(xPower, yPower, turnPower, 1, 1);
        Module.MotorPowers modulePowersBR = moduleBR.calculateRawMotorPowers(xPower, yPower, turnPower, 1, 1);
        Module.MotorPowers modulePowersBL = moduleBL.calculateRawMotorPowers(xPower, yPower, turnPower, 1, -1);


        // run module calculations, determine motor power scale
        double motorPowerScale = 1 / maxMotorMag(modulePowersFR, modulePowersFL, modulePowersBR, modulePowersBL);

        // if there's no need to scale then don't scale
        if (motorPowerScale > 1) {
            motorPowerScale = 1.0;
        }

        //apply motor powers in modules with scale factor
        moduleFR.applyPowers(modulePowersFR, motorPowerScale);
//        moduleFL.applyPowers(modulePowersFL, motorPowerScale);
//        moduleBR.applyPowers(modulePowersBR, motorPowerScale);
//        moduleBL.applyPowers(modulePowersBL, motorPowerScale);
    }

    //returns the max motor magnitude from the return objects
    public double maxMotorMag(Module.MotorPowers module1Powers, Module.MotorPowers module2Powers,
                              Module.MotorPowers module3Powers, Module.MotorPowers module4Powers) {

        double max = module1Powers.getMaxMag();
        if (module2Powers.getMaxMag() > max) {
            max = module2Powers.getMaxMag();
        }
        if (module3Powers.getMaxMag() > max) {
            max = module3Powers.getMaxMag();
        }
        if (module4Powers.getMaxMag() > max) {
            max = module4Powers.getMaxMag();
        }

        return max;
    }
    public double getHeadingFL(){
        return moduleFL.calculateHeading(1,1);
    }
    public double getHeadingFR(){
        return moduleFR.calculateHeading(-1,1);
    }
    public double getHeadingBL(){
        return moduleBL.calculateHeading(1,-1);
    }
    public double getHeadingBR(){
        return moduleBR.calculateHeading(1,1);
    }

    public double getFRP1(){
        return moduleFR.getP1();
    }
    public double getFRP2(){
        return moduleFR.getP2();
    }
    public double getFLP1(){
        return moduleFL.getP1();
    }
    public double getFLP2(){
        return moduleFL.getP2();
    }
    public double getBRP1(){
        return moduleBR.getP1();
    }
    public double getBRP2(){
        return moduleBR.getP2();
    }
    public double getBLP1(){
        return moduleBL.getP1();
    }
    public double getBLP2(){
        return moduleBL.getP2();
    }




}

