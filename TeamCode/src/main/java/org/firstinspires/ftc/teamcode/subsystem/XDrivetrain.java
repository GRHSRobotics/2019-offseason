package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.math.PDController;

//TODO make a way to have IMU initialization without extending LinearOpMode
//this is a messy and temporary solution
//extending an OpMode class in a hardware class is no bueno
public class XDrivetrain extends LinearOpMode {

    public DcMotorEx front;
    public DcMotorEx back;
    public DcMotorEx left;
    public DcMotorEx right;

    public BNO055IMU imu; //REV Hub

    //DRIVETRAIN CONSTANTS
    public static final double MAX_VELOCITY = 1; //inches per second
    public static final double MAX_ACCELERATION = 0.15; //inches per second^2
    public static final double WHEEL_RADIUS = 2; //inches
    public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI; //inches
    public static final double ROBOT_DIAMETER = 18; //inches
    public static final double COUNTS_PER_ROTATION = 280;
    public static final double COUNTS_PER_INCH = COUNTS_PER_ROTATION / WHEEL_CIRCUMFERENCE;

    public static final double kP_DRIVE = 0.01;
    public static final double kD_DRIVE = 0.1; //until constants are properly determined
    public static final double kP_TURN = 0.01;
    public static final double kD_TURN = 0.1;

    public static final double DRIVE_ERROR_THRESHOLD = 0.3; //max acceptable error in position after a movement, in inches


    public XDrivetrain(HardwareMap hardwareMap){
        this.front = hardwareMap.get(DcMotorEx.class, "front");
        this.back = hardwareMap.get(DcMotorEx.class, "back");
        this.left = hardwareMap.get(DcMotorEx.class, "left");
        this.right = hardwareMap.get(DcMotorEx.class, "right");

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

    public void initIMU(HardwareMap hardwareMap){ //we do this one separately so we don't waste time initializing the IMU when we don't need it
        //DEFINE REV HUB IMU
        this.imu = hardwareMap.get(BNO055IMU.class, "hub1imu");

        //SET IMU PARAMETERS
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        //INIT IMU
        this.imu.initialize(parameters);

        telemetry.addData("Mode", "Calibrating Gyro");

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !this.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        //gyroHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //only keeping this here as a reference on how to get gyro heading

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());

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
    /*
    public void setPowerEx(double forwardPower, double strafePower){

        front.setVelocity(strafePower, );

    }
    */

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

    /**
     * Drive forward and sideways a certain number of inches.
     * This method is velocity and acceleration limited and utilizes a PD loop to determine target velocity
     *
     * @param forwardInches the forward distance to be travelled by the robot
     * @param strafeInches the side to side distance to be travelled by the robot, where to the right is positive
     * @param speedFraction a multiplier between 0 and 1 for what fraction of the robot's max speed can be used
     * @param maxTimeS the maximum time before the method terminates, in seconds
     */
    public void encoderDrive(double forwardInches, double strafeInches, double speedFraction, double maxTimeS){

        //ensures that motor mode is set the way we want it
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime timer = new ElapsedTime();

        PDController forwardController = new PDController(kP_DRIVE, kD_DRIVE, timer); //constants are the same because x drive is same in both directions
        PDController strafeController = new PDController(kP_DRIVE, kD_DRIVE, timer);

        int forwardTicks = (int)(forwardInches * COUNTS_PER_INCH);
        int strafeTicks = (int)(strafeInches * COUNTS_PER_INCH);

        front.setTargetPosition(front.getCurrentPosition() + strafeTicks);
        back.setTargetPosition(back.getCurrentPosition() + strafeTicks);
        left.setTargetPosition(left.getCurrentPosition() + forwardTicks);
        right.setTargetPosition(right.getCurrentPosition() + forwardTicks);

        double forwardVelocityTICKS = 0; //radians per second
        double strafeVelocityTICKS = 0; //radians per second
        double previousForwardVelocityTICKS = 0;
        double previousStrafeVelocityTICKS = 0;

        double forwardError; //inches
        double strafeError;

        double time;
        double previousTime = 0;

        boolean forwardTargetReached = false;
        boolean strafeTargetReached = false;

        while(opModeIsActive() && (!forwardTargetReached || !strafeTargetReached)){

            time = timer.seconds();

            forwardError = (left.getTargetPosition() - left.getCurrentPosition()) / COUNTS_PER_INCH;
            strafeError = (front.getTargetPosition() - front.getCurrentPosition()) / COUNTS_PER_INCH;

            forwardVelocityTICKS = forwardController.getOutput(forwardError) * COUNTS_PER_INCH;
            strafeVelocityTICKS = strafeController.getOutput(strafeError) * COUNTS_PER_INCH;


            //keep change in velocity to within a sane range AKA acceleration limiting
            //counts per inch included because max accel constant is in linear units

            if(Math.abs(forwardVelocityTICKS - previousForwardVelocityTICKS) / ((time-previousTime)* COUNTS_PER_INCH) > MAX_ACCELERATION){
                if(forwardVelocityTICKS > previousForwardVelocityTICKS){

                    //if PD controller says to increase velocity, add to previous velocity
                    forwardVelocityTICKS = previousForwardVelocityTICKS + (MAX_ACCELERATION * (time-previousTime)) * COUNTS_PER_INCH;

                } else if(forwardVelocityTICKS < previousForwardVelocityTICKS){

                    //if PD controller says to decrease velocity, subtract from previous velocity
                    forwardVelocityTICKS = previousForwardVelocityTICKS - (MAX_ACCELERATION * (time-previousTime)) * COUNTS_PER_INCH;

                }
            }

            if(Math.abs(strafeVelocityTICKS - previousStrafeVelocityTICKS) / ((time-previousTime)* COUNTS_PER_INCH) > MAX_ACCELERATION){
                if(strafeVelocityTICKS > previousStrafeVelocityTICKS){

                    //if PD controller says to increase velocity, add to previous velocity
                    strafeVelocityTICKS = previousStrafeVelocityTICKS + (MAX_ACCELERATION * (time-previousTime)) * COUNTS_PER_INCH;

                } else if(strafeVelocityTICKS < previousStrafeVelocityTICKS){

                    //if PD controller says to decrease velocity, subtract from previous velocity
                    strafeVelocityTICKS = previousStrafeVelocityTICKS - (MAX_ACCELERATION * (time-previousTime)) * COUNTS_PER_INCH;

                }
            }

            //assign velocities to motors
            front.setVelocity(strafeVelocityTICKS);
            back.setVelocity(strafeVelocityTICKS);
            left.setVelocity(forwardVelocityTICKS);
            right.setVelocity(forwardVelocityTICKS);

            //check if targets are reached
            if(forwardError < DRIVE_ERROR_THRESHOLD){
                forwardTargetReached = true;
            }
            if(strafeError < DRIVE_ERROR_THRESHOLD){
                strafeTargetReached = true;
            }

            //set variables for next loop
            previousTime = time;
            previousForwardVelocityTICKS = forwardVelocityTICKS;
            previousStrafeVelocityTICKS = strafeVelocityTICKS;

        }


    }


    //this is here because it needs to be
    //temporary solution until we can find a way to not extend LinearOpMode
    @Override
    public void runOpMode(){}
}
