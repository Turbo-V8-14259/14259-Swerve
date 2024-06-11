package org.firstinspires.ftc.teamcode;

//imports
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



// one wheel module
public class Module {

    // drive motors
    private DcMotor motor1;
    private DcMotor motor2;

    // home sensor
    private RevColorSensorV3 homeSensor;

    // angle to go to when the robot spins
    private final double TURN_HEADING;

    // angle when the home sensor triggers
    private final double HOME_HEADING;

    // power to home module at, polarity matters
    private final double HOME_POWER;

    // has the module homed yet
    private boolean homed;

    // home sensor threshold
    private final double HOME_THRESHOLD;




    // return type so both motor powers can be returned
    public class MotorPowers {
        public double motor1Power;
        public double motor2Power;

        public MotorPowers(double motor1Power, double motor2Power) {
            this.motor1Power = motor1Power;
            this.motor2Power = motor2Power;
        }

        //returns the max magnitude
        public double getMaxMag() {

            double max = Math.abs(motor1Power);
            if(Math.abs(motor2Power) > Math.abs(motor1Power)) { max = Math.abs(motor2Power); }

            return max;
        }
    }


    // CONSTRUCTOR
    public Module(DcMotor motor1, DcMotor motor2, RevColorSensorV3 homeSensor,
                  double turnHeading, double homeHeading, double homePower){

        // set hardware
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.homeSensor = homeSensor;

        // reverse motor2 direction since its gearing is inverted
//        motor2.setDirection(DcMotor.Direction.REVERSE);
//        motor1.setDirection(DcMotor.Direction.REVERSE);
        //both forwrad worked
        //both reverse sus but worked


        // idk if you need this but it makes sense
        homeSensor.initialize();

        // set input values
        this.TURN_HEADING = turnHeading;
        this.HOME_HEADING = homeHeading;
        this.HOME_POWER = homePower;

        // initial values
        homed = false;
        HOME_THRESHOLD = 10;
    }







    // returns both motor powers but they could be up to 2.0
    public MotorPowers calculateRawMotorPowers(double xPowerRobot, double yPowerRobot, double turnPowerRobot) {

        // calculate module heading
        double currentHeading = calculateHeading(-motor1.getCurrentPosition(), motor2.getCurrentPosition());






        //module vector to generate (frame coordinates)
        //but guys swerve math is really hard
        double xPowerFrame = xPowerRobot + (turnPowerRobot * Math.cos(TURN_HEADING));
        double yPowerFrame = yPowerRobot + (turnPowerRobot * Math.sin(TURN_HEADING));

        //total power vector
        double powerMagnitude = Math.hypot(xPowerFrame, yPowerFrame);

        //power vector direction in module coordinates
        double powerDirectionModule = Math.atan2(yPowerFrame, xPowerFrame) - currentHeading;
        powerDirectionModule = angleWrap(powerDirectionModule);





        //find module rotation and wheel powers
        double wheelPower = powerMagnitude * Math.sin(powerDirectionModule);
        double modulePower = powerMagnitude * Math.cos(powerDirectionModule);




        //return motor powers
        return calculatePowers(modulePower, wheelPower);
    }






    // homes the module and resets encoders there, returns true when done
    public boolean homeModule() {

        // if we're in a home position and haven't homed yet
        if(homeSensor.getDistance(DistanceUnit.MM) < HOME_THRESHOLD && !homed) {

            // reset the encoders
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // record that we're homed so it stops moving
            homed = true;
        }

        // motors move by default
        double motor1Power = HOME_POWER;
        double motor2Power = HOME_POWER;

        //if we've homed, don't continue moving motors
        if(homed) {
            motor1Power = 0;
            motor2Power = 0;

            //start updating heading now that its homed
            calculateHeading(-motor1.getCurrentPosition(), motor2.getCurrentPosition());
        }


        //apply the powers
        motor1.setPower(motor1Power);
        motor2.setPower(motor2Power);

        //return true if done homing
        return homed;
    }








    //updates module heading, angle wraps to +-180
    public double calculateHeading(int motor1Encoder, int motor2Encoder) {

        //calculate constant with gear ratio or just tune lol
        // subtract home heading since that's where the encoders are reset
        double heading = ((motor1Encoder + motor2Encoder) * 0.01527) - HOME_HEADING;

        //angle wrap
        return angleWrap(heading);
    }


    //angle wrap to [-180, 180)
    public double angleWrap(double inputRad) {
        //modulo being sussy
        int inputSign  = 1;
        if(inputRad < 0) {
            inputSign = -1;
        }
        inputRad = Math.abs(inputRad);
        inputRad %= 2 * Math.PI;
        inputRad -= Math.PI;

        return inputRad * inputSign;
    }


    //calculates motor powers, diffy is so much harder to program fr
    public MotorPowers calculatePowers(double modulePower, double wheelPower) {

        return new MotorPowers(modulePower + wheelPower, modulePower - wheelPower);
    }


    //applies the motor powers from the input, scales by scale input
    public void applyPowers(MotorPowers rawPowers, double scale) {
        motor1.setPower(rawPowers.motor1Power * scale);
        motor2.setPower(rawPowers.motor2Power * scale);
    }

    //returns if module is homed
    public boolean getHomed() {
        return homed;
    }

}

