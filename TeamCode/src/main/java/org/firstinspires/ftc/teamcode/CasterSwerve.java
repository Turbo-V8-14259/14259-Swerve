package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class CasterSwerve {


    Module moduleFR;
    Module moduleFL;
    Module moduleBR;
    Module moduleBL;


    public CasterSwerve(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4,
                        DcMotor motor5, DcMotor motor6, DcMotor motor7, DcMotor motor8) {
        //setup modules
        moduleFR = new Module(motor1, motor2 , Math.toRadians(45), Math.toRadians(0));
        moduleFL = new Module(motor3, motor4, Math.toRadians(-45), Math.toRadians(0));
        moduleBR = new Module(motor5, motor6, Math.toRadians(135), Math.toRadians(0));
        moduleBL = new Module(motor7, motor8, Math.toRadians(-135), Math.toRadians(0));
    }


    public void drive(double straightPower, double strafePower, double spinPower) {
        double xPower = strafePower;
        double yPower = straightPower;
        double turnPower = spinPower;


        // calculate raw motor powers for each module
        Module.MotorPowers modulePowersFR = moduleFR.calculateRawMotorPowers(xPower, yPower, turnPower, -1, 1, -1,-1);
        Module.MotorPowers modulePowersFL = moduleFL.calculateRawMotorPowers(xPower, yPower, turnPower, 1, -1, 1,1);//tune
        Module.MotorPowers modulePowersBR = moduleBR.calculateRawMotorPowers(xPower, yPower, turnPower, -1, 1, -1, 1);
        Module.MotorPowers modulePowersBL = moduleBL.calculateRawMotorPowers(xPower, yPower, turnPower, 1, -1, 1, 1);


        // run module calculations, determine motor power scale
        double motorPowerScale = 1 / maxMotorMag(modulePowersFR, modulePowersFL, modulePowersBR, modulePowersBL);

        // if there's no need to scale then don't scale
        if (motorPowerScale > 1) {
            motorPowerScale = 1.0;
        }

        //apply motor powers in modules with scale factor
        moduleFR.applyPowers(modulePowersFR, motorPowerScale);
        moduleFL.applyPowers(modulePowersFL, motorPowerScale);
        moduleBR.applyPowers(modulePowersBR, motorPowerScale);
        moduleBL.applyPowers(modulePowersBL, motorPowerScale);
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

    //make sure the signs match what u initialize it with, returns the heading of the module
    public double getHeadingFL(){
        return moduleFL.calculateHeading(1,-1);
    }
    public double getHeadingFR(){
        return moduleFR.calculateHeading(-1,1);
    }
    public double getHeadingBL(){
        return moduleBL.calculateHeading(1,-1);
    }
    public double getHeadingBR(){
        return moduleBR.calculateHeading(-1,1);
    }



}

