package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrivetrain {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    //CONSTANTS
    public static final double COUNTS_PER_REVOLUTION = 560; //assuming HD Hex 20:1
    public static final double WHEEL_RADIUS = 2; //inches
    public static final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (2 * Math.PI * WHEEL_RADIUS); //counts per rev divided by inches per rev


    public MecanumDrivetrain(HardwareMap hardwareMap, Telemetry telemetry){
        this.frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        this.frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        this.backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        this.backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        //reverse right motors so that the forward direction is the same way for all motors
        this.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        this.frontRight.setDirection(DcMotor.Direction.REVERSE); //assumption for which way is forwards made here
        this.backLeft.setDirection(DcMotor.Direction.FORWARD);
        this.backRight.setDirection(DcMotor.Direction.REVERSE);

        //set zero power mode and run mode
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Drivetrain: ", "Initialized");
        telemetry.update();

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
     * @param rotationSpeed The desired rate of change of robot direction [-1, 1]
     * @param angleUnit The unit of the inputted angle, either in degrees or radians
     */
    public void setPower(double speed, double direction, double rotationSpeed, AngleUnit angleUnit){

        double angle;
        double clampedSpeed;
        double clampedRotationSpeed;

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
        if(rotationSpeed > 1){
            clampedRotationSpeed = 1;
        } else if(rotationSpeed < -1){
            clampedRotationSpeed = -1;
        } else{
            clampedRotationSpeed = rotationSpeed;
        }

        //calculate raw motor powers, see team resources for math explanation
        double rawPowerFL = clampedSpeed * Math.sin(-1*angle + Math.PI/4) - clampedRotationSpeed;
        double rawPowerFR = clampedSpeed * Math.cos(-1*angle + Math.PI/4) + clampedRotationSpeed;
        double rawPowerBL = clampedSpeed * Math.cos(-1*angle + Math.PI/4) - clampedRotationSpeed;
        double rawPowerBR = clampedSpeed * Math.sin(-1*angle + Math.PI/4) + clampedRotationSpeed;

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
