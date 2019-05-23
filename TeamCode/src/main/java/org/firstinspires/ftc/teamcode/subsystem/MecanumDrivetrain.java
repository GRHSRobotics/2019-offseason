package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.math.AngleUnit;

public class MecanumDrivetrain {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public MecanumDrivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        //reverse right motors so that the forward direction is the same way for all motors
        this.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        this.frontRight.setDirection(DcMotor.Direction.REVERSE); //assumption for which way is forwards made here
        this.backLeft.setDirection(DcMotor.Direction.FORWARD);
        this.backRight.setDirection(DcMotor.Direction.REVERSE);

        //set zero power mode and run mode
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setRunMode(DcMotor.RunMode runMode){
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(zeroPowerBehavior);
        backRight.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     *
     * @param speed The desired speed of robot movement in the range [-1, 1]
     * @param direction The desired direction of robot travel in the range [0, 2pi]
     * @param angleSpeed The desired rate of change of robot direction [-1, 1]
     * @param angleUnit The unit of the inputted angle, either in degrees or radians
     */
    public void setPower(double speed, double direction, double angleSpeed, AngleUnit angleUnit){

        double angle;
        double clampedSpeed;
        double clampedAngleSpeed;

        //convert to radians if necessary
        if(angleUnit == AngleUnit.DEGREES){
            angle = Math.toRadians(direction);
        } else {
            angle = direction;
        }

        //clamp robot speed if necessary
        if(speed > 1){
           clampedSpeed = 1;
        } else if(speed < -1){
            clampedSpeed = -1;
        } else {
            clampedSpeed = speed;
        }

        //clamp direction change speed if necessary
        if(angleSpeed > 1){
            clampedAngleSpeed = 1;
        } else if(angleSpeed < -1){
            clampedAngleSpeed = -1;
        } else{
            clampedAngleSpeed = angleSpeed;
        }

        //calculate raw motor powers, see team resources for math explanation
        double rawPowerFL = clampedSpeed * Math.sin(-1*angle + Math.PI/4) - clampedAngleSpeed;
        double rawPowerFR = clampedSpeed * Math.cos(-1*angle + Math.PI/4) + clampedAngleSpeed;
        double rawPowerBL = clampedSpeed * Math.cos(-1*angle + Math.PI/4) - clampedAngleSpeed;
        double rawPowerBR = clampedSpeed * Math.sin(-1*angle + Math.PI/4) + clampedAngleSpeed;

        //reduce all motor powers to a max of 1 while maintaining the ratio between them
        double highestPower = Math.max(Math.max(rawPowerFL, rawPowerFR), Math.max(rawPowerBL, rawPowerBR)); //contains the highest of the 4 raw powers

        double powerFL = rawPowerFL / highestPower;
        double powerFR = rawPowerFR / highestPower;
        double powerBL = rawPowerBL / highestPower;
        double powerBR = rawPowerBR / highestPower;

        //set final motor powers
        frontLeft.setPower(powerFL);
        frontRight.setPower(powerFR);
        backLeft.setPower(powerBL);
        backRight.setPower(powerBR);

    }

}
