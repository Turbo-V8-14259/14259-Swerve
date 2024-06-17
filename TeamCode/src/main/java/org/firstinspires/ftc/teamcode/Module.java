package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;



// one wheel module
public class Module {

    private DcMotor motor1;
    private DcMotor motor2;


    // angle to go to when the robot spins
    private double TURN_HEADING = 0;
    //angle the modules start at
    private double HOME_HEADING = 0;

    private double heading = 0;




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
    public Module(DcMotor motor1, DcMotor motor2,
                  double turnHeading, double homeHeading) {

        // set hardware
        this.motor1 = motor1;
        this.motor2 = motor2;
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // set input values
        this.TURN_HEADING = turnHeading;
        this.HOME_HEADING = homeHeading;
    }







    // returns both motor powers but they could be up to 2.0
    public MotorPowers calculateRawMotorPowers(double xPowerRobot, double yPowerRobot, double turnPowerRobot, int encoderSine1, int encoderSine2, int sinModule1, int sinModule2) {

        // calculate module heading
        double currentHeading = calculateHeading(encoderSine1, encoderSine2);

        //module vector to generate (frame coordinates)
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
        return calculatePowers(modulePower * sinModule2, wheelPower * sinModule1);
    }




    //updates module heading, angle wraps to +-180
    public double calculateHeading(int encoderSine1, int encoderSine2) {
        // subtract home heading since that's where the encoders are reset
        //0.01527 * 1.015 * 1.000314 is the gear ratio constant for the encoders
        heading = (((motor1.getCurrentPosition() * encoderSine1) + motor2.getCurrentPosition() * encoderSine2) * 0.01527 * 1.015 * 1.000314) - HOME_HEADING;

        return angleWrap(heading);
    }


    //angle wrap to [-180, 180)
    public double angleWrap(double inputRad) {
        double inputSign  = Math.signum(inputRad);
        inputRad = Math.abs(inputRad);
        inputRad %= 2 * Math.PI;
        inputRad -= Math.PI;

        return inputRad * inputSign;
    }


    //calculates motor powers, diffy is so much harder to program fr
    public MotorPowers calculatePowers(double modulePower, double wheelPower) {

        return new MotorPowers(modulePower - wheelPower, modulePower + wheelPower);
    }


    //applies the motor powers from the input, scales by scale input
    public void applyPowers(MotorPowers rawPowers, double scale) {
        motor1.setPower(rawPowers.motor1Power * scale);
        motor2.setPower(rawPowers.motor2Power * scale);
    }



}

