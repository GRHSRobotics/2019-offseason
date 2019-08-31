package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.math.PIDController;

//TODO make a way to have IMU initialization without extending LinearOpMode
//this is a messy and temporary solution
//extending an OpMode class in a hardware class is no bueno
public class XDrivetrain {

    public DcMotor front;
    public DcMotor back;
    public DcMotor left;
    public DcMotor right;

    public BNO055IMU imu; //REV Hub

    //DRIVETRAIN CONSTANTS
    public static final double MAX_VELOCITY = 1; //power
    public static final double MAX_ACCELERATION = 0.15; //power per second
    public static final double WHEEL_RADIUS = 1.5; //inches
    public static final double WHEEL_CIRCUMFERENCE = 3 * Math.PI; //inches
    public static final double ROBOT_DIAMETER = 18; //inches
    public static final double COUNTS_PER_ROTATION = 1120;
    public static final double COUNTS_PER_INCH = COUNTS_PER_ROTATION / WHEEL_CIRCUMFERENCE;

    public static final double kP_DRIVE = 0.8;
    public static final double kI_DRIVE = 0;
    public static final double kD_DRIVE = 10; //until constants are properly determined
    public static final double kP_TURN = 0.01;
    public static final double kI_TURN = 0;
    public static final double kD_TURN = 0.1;

    public static final double DRIVE_ERROR_THRESHOLD = 0.5; //max acceptable error in position after a movement, in inches


    public XDrivetrain(HardwareMap hardwareMap){
        this.front = hardwareMap.get(DcMotor.class, "front");
        this.back = hardwareMap.get(DcMotor.class, "back");
        this.left = hardwareMap.get(DcMotor.class, "left");
        this.right = hardwareMap.get(DcMotor.class, "right");

        //change motor directions so that positive powers move the right way
        this.front.setDirection(DcMotor.Direction.REVERSE);
        this.back.setDirection(DcMotor.Direction.FORWARD);
        this.left.setDirection(DcMotor.Direction.REVERSE);
        this.right.setDirection(DcMotor.Direction.FORWARD);

        //assign runmode and zero power behavior
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }


    public void setRunMode(DcMotor.RunMode runMode){
        front.setMode(runMode);
        back.setMode(runMode);
        left.setMode(runMode);
        right.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        front.setZeroPowerBehavior(zeroPowerBehavior);
        back.setZeroPowerBehavior(zeroPowerBehavior);
        left.setZeroPowerBehavior(zeroPowerBehavior);
        right.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * This method simply assigns the given powers to the drive motors.
     * In the event that one or both of them is outside the proper range, it does not attempt to keep the scaling between them.
     * It simply clamps the powers to the range [-1,1]
     *
     * @param forwardPower power in the y direction in the range [-1,1]
     * @param strafePower power in the x direction in the range [-1,1]
     */
    public void setPowerRectangular(double forwardPower, double strafePower){
        double clampedForwardPower;
        double clampedStrafePower;

        if(forwardPower > 1){
            clampedForwardPower = 1;
        } else if(forwardPower < -1){
            clampedForwardPower = -1;
        } else{
            clampedForwardPower = forwardPower;
        }

        if(strafePower > 1){
            clampedStrafePower = 1;
        } else if(strafePower < -1){
            clampedStrafePower = -1;
        } else{
            clampedStrafePower = strafePower;
        }

        front.setPower(clampedStrafePower);
        back.setPower(clampedStrafePower);
        left.setPower(clampedForwardPower);
        right.setPower(clampedForwardPower);


    }


    /**
     *
     * @param speed The desired speed of robot movement in the range [-1, 1]
     * @param direction The desired direction of robot travel in the range [0, 2pi]
     * @param rotationSpeed The desired rate of change of robot direction [-1, 1]
     * @param angleUnit The unit of the inputted angle, either in degrees or radians
     */
    public void setPowerPolar(double speed, double direction, double rotationSpeed, AngleUnit angleUnit){

        double angle;
        double clampedSpeed;
        double clampedRotationSpeed;

        if(angleUnit == AngleUnit.RADIANS){
            angle = direction;
        } else {
            angle = Math.toRadians(direction);
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

        double rawStrafeTranslation = clampedSpeed * Math.cos(angle);
        double rawForwardTranslation = clampedSpeed * Math.sin(angle);

        double rawFrontPower = rawStrafeTranslation - clampedRotationSpeed;
        double rawBackPower = rawStrafeTranslation + clampedRotationSpeed;
        double rawLeftPower = rawForwardTranslation - clampedRotationSpeed;
        double rawRightPower = rawForwardTranslation + clampedRotationSpeed;

        //reduce all motor powers to a max of 1 while maintaining the ratio between them
        double highestPower = Math.max(Math.max(rawFrontPower, rawBackPower), Math.max(rawLeftPower, rawRightPower)); //contains the highest of the 4 raw powers

        double frontPower = rawFrontPower / highestPower;
        double backPower = rawBackPower / highestPower;
        double leftPower = rawLeftPower / highestPower;
        double rightPower = rawRightPower / highestPower;

        //set final powers
        front.setPower(frontPower);
        back.setPower(backPower);
        left.setPower(leftPower);
        right.setPower(rightPower);


    }


}
