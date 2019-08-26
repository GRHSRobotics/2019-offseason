package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {

    double kP;
    double kI;
    double maxI = 1;
    double kD;
    ElapsedTime timer;
    Telemetry telemetry;
    double previousTime;
    double lastInput = 0;
    double errorSum = 0;


    public PIDController(double kP, double kI, double kD, ElapsedTime timer, Telemetry telemetry){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.timer = timer;
        this.previousTime = timer.seconds();
        this.telemetry = telemetry;
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

        //then we get the integral part of the output power
        //this is basically a sum of the error over time and allows us to reduce error where the system is hovering right below the desired output (steady state error)
        errorSum += error * (timer.seconds() - previousTime);
        double integralCorrection = kI * errorSum;

        if(Math.abs(integralCorrection) >= maxI){
            //prevent integral windup, a case where integral term builds up too much over time
            integralCorrection = Math.copySign(maxI, integralCorrection); //keeps the sign of the I term the same
        }

        //last we get the derivative part of the output power
        //in other words, this is basically how fast the error is changing: a faster changing error will lead to greater D term

        double derivativeCorrection;

        if((timer.seconds() - previousTime) > 3){
            //this catches special cases where the calculation for the slope of the input curve would be wonky
            derivativeCorrection = 0;
        } else {
            derivativeCorrection = kD * ((input - lastInput) / (timer.seconds() - previousTime));
        }

        //set current error as comparison point for next time this method runs
        lastInput = input;

        //to see what's going on
        //telemetry.addData("Proportional Term:", proportionalCorrection);
        //telemetry.addData("integral Term:", integralCorrection);
        //telemetry.addData("Derivative Term:", derivativeCorrection);

        //put it all together
        //derivative term is subtracted since we want the output to go down if its changing too fast
        return proportionalCorrection + integralCorrection - derivativeCorrection;

    }

    public void setCoefficients(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setMaxI(double maxI){
        this.maxI = maxI;
    }

}
