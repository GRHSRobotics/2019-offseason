package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PDController {

    double kP;
    double kD;
    ElapsedTime timer;
    double previousTime;
    double lastError = 0;


    public PDController(double kP, double kD, ElapsedTime timer){
        this.kP = kP;
        this.kD = kD;
        this.timer = timer;
        this.previousTime = timer.seconds();
    }

    /**
     *
     * @param input The actual state of the system being controlled, usually recorded by sensors
     * @param setPoint The desired state of the system
     * @return The motor power that will minimize the time it takes to reach the setpoint given that this method is called repeatedly and the P and D coefficients have been tuned.
     */
    public double getOutput(double input, double setPoint){

        double error = setPoint - input; //positive error means that the setpoint has not yet been reached, and will output a positive power

        //first we get the proportional term of the output power
        double proportionalCorrection = kP * error;

        //then we get the derivative part of the output power
        //in other words, this is basically how fast the error is changing: a faster changing error will lead to greater D term

        double derivativeCorrection;

        if(lastError == 0 || (timer.seconds() - previousTime) > 3){
            //this catches special cases where the calculation for the slope of the error curve would be wonky
            //AKA when there is no comparison point for the error or when the method hasn't been run often enough to have a comparison for time
            //mainly just for when the method runs the first time
            derivativeCorrection = 0;
        } else {
            derivativeCorrection = kD * ((error - lastError) / (timer.seconds() - previousTime));
        }

        //set current error as comparison point for next time this method runs
        lastError = error;

        //put it all together
        return proportionalCorrection + derivativeCorrection;

    }

    public void setCoefficients(double kP, double kD){
        this.kP = kP;
        this.kD = kD;
    }

}
