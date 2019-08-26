package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.math.FieldCentricNavigator;
import org.firstinspires.ftc.teamcode.math.PIDController;

public class AutonomousBase extends LinearOpMode {

    //AUTONOMOUS CONSTANTS
    double DRIVE_POSITION_ERROR_THRESHOLD = 0.3; //inches
    double DRIVE_MAX_ACCEL = 0.3; //power per second
    double INITIAL_ROBOT_ANGLE_RADIANS = 0;

    double DRIVE_KP = 0.8; //guesses bc no real numbers yet
    double DRIVE_KI = 0.01;
    double DRIVE_KD = 0.01;

    //TODO put all essential/basic auton methods here
    //assumes mecanum drivetrain for simplicity

    Robot robot;

    public void setInitialRobotAngle(double angle, AngleUnit angleUnit){
        if(angleUnit == AngleUnit.RADIANS){
            INITIAL_ROBOT_ANGLE_RADIANS = angle;
        } else {
            INITIAL_ROBOT_ANGLE_RADIANS = Math.toRadians(angle);
        }
    }


    /**
     *  Basic omnidirectional encoder-based translational movement for mecanum drivetrain
     * @param distance magnitude of translation in inches
     * @param angle ROBOT CENTRIC direction of translation, where 0 is forwards, positive angles are left, and negative angles are right (-180, 180)
     * @param angleUnit unit of inputted angle, either radians or degrees
     * @param maxTimeS time before method is cut off
     */
    public void drive(double distance, double angle, AngleUnit angleUnit, double maxTimeS){

        //motor mode stuff
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double target = distance;
        double currentPosition = 0;

        double initialFL = robot.drivetrain.frontLeft.getCurrentPosition();
        double initialFR = robot.drivetrain.frontRight.getCurrentPosition();
        double initialBL = robot.drivetrain.backLeft.getCurrentPosition();
        double initialBR = robot.drivetrain.backRight.getCurrentPosition();

        double theta;

        //change angle so that 0 is due right
        if(angleUnit == AngleUnit.DEGREES){
            theta = Math.toRadians(angle + 90);
        } else {
            theta = angle + Math.PI/2;
        }

        ElapsedTime timer = new ElapsedTime();


        //no need to add negative cases for power or position because these will always be positive due to the polar nature of the algorithm
        while(target - currentPosition > DRIVE_POSITION_ERROR_THRESHOLD && timer.seconds() < maxTimeS && opModeIsActive()){

            //see https://ftcforum.usfirst.org/forum/ftc-technology/50373-mecanum-encoder-algorithm
            //mecanum specific displacement formula
            double currentX = ((robot.drivetrain.frontLeft.getCurrentPosition() - initialFL + robot.drivetrain.backRight.getCurrentPosition() - initialBR)
                    - (robot.drivetrain.frontRight.getCurrentPosition() - initialFR + robot.drivetrain.backLeft.getCurrentPosition() - initialBL))
                    / (4 * robot.drivetrain.COUNTS_PER_INCH);

            double currentY = ((robot.drivetrain.frontLeft.getCurrentPosition() - initialFL) + (robot.drivetrain.frontRight.getCurrentPosition() - initialFR)
                    + (robot.drivetrain.backLeft.getCurrentPosition() - initialBL) + (robot.drivetrain.backRight.getCurrentPosition() - initialBR))
                    / (4 * robot.drivetrain.COUNTS_PER_INCH);

            //take these 2 sides (components) of a right triangle and find the hypotenuse (total displacement)
            currentPosition = Math.sqrt(Math.pow(currentX, 2) + Math.pow(currentY, 2));

            robot.drivetrain.setPower(0.5, theta, 0, AngleUnit.RADIANS);

        }

        //stop after movement is complete
        robot.drivetrain.setPower(0, theta, 0, AngleUnit.RADIANS);

    }


    /**
     * Deluxe omnidirectional translation for mecanum, including PID speed control, active rotation limiting (maybe), field centric input, and acceleration limiting
     * @param distance magnitude of translation in inches
     * @param angle direction of translation, where 0 is forwards, positive angles are left, and negative angles are right (-180, 180) or radian equivalent
     * @param angleUnit unit of inputted angle, either radians or degrees
     * @param maxTimeS time before method is cut off
     */
    public void fieldCentricDrive(double distance, double angle, AngleUnit angleUnit, double maxTimeS){

        //motor mode stuff
        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double target = distance; //put into a variable in case we ever need to do math on it or something
        double currentPosition = 0;

        double initialFL = robot.drivetrain.frontLeft.getCurrentPosition();
        double initialFR = robot.drivetrain.frontRight.getCurrentPosition();
        double initialBL = robot.drivetrain.backLeft.getCurrentPosition();
        double initialBR = robot.drivetrain.backRight.getCurrentPosition();

        //for use as a reference point to make sure that robot is holding heading through the translation
        double initialHeading = robot.gyroscope.getAngle(AngleUnit.DEGREES);

        double theta;
        if(angleUnit == AngleUnit.DEGREES){
            theta = Math.toRadians(angle);
        } else {
            theta = angle;
        }

        //convert the given field-centric angle into a robot-centric one that can be fed to the motors
        FieldCentricNavigator fieldCentricNavigator = new FieldCentricNavigator(INITIAL_ROBOT_ANGLE_RADIANS, theta);
        double robotCentricAngle = fieldCentricNavigator.fieldToRobotCentric(robot.gyroscope.getAngle(AngleUnit.RADIANS));

        //change angle so that 0 is due right
        robotCentricAngle += Math.PI/2;

        ElapsedTime timer = new ElapsedTime();


        //no need to add negative cases for power or position because these will always be positive due to the polar nature of the algorithm
        while(target - currentPosition > DRIVE_POSITION_ERROR_THRESHOLD && timer.seconds() < maxTimeS && opModeIsActive()){

            //see https://ftcforum.usfirst.org/forum/ftc-technology/50373-mecanum-encoder-algorithm
            //mecanum specific displacement formula
            double currentX = ((robot.drivetrain.frontLeft.getCurrentPosition() - initialFL + robot.drivetrain.backRight.getCurrentPosition() - initialBR)
                    - (robot.drivetrain.frontRight.getCurrentPosition() - initialFR + robot.drivetrain.backLeft.getCurrentPosition() - initialBL))
                    / (4 * robot.drivetrain.COUNTS_PER_INCH);

            double currentY = ((robot.drivetrain.frontLeft.getCurrentPosition() - initialFL) + (robot.drivetrain.frontRight.getCurrentPosition() - initialFR)
                    + (robot.drivetrain.backLeft.getCurrentPosition() - initialBL) + (robot.drivetrain.backRight.getCurrentPosition() - initialBR))
                    / (4 * robot.drivetrain.COUNTS_PER_INCH);

            //take these 2 sides (components) of a right triangle and find the hypotenuse (total displacement)
            currentPosition = Math.sqrt(Math.pow(currentX, 2) + Math.pow(currentY, 2));

            //PID controller is most effective as the robot is approaching its target
            PIDController powerController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, timer, telemetry);

            //make it take the minimum of a straight line sloping upwards and the PID controller output
            //this ensures that the acceleration at the beginning of the movement is limited without affecting the PID controller later in the movement
            double translationPower = Math.min(DRIVE_MAX_ACCEL * timer.seconds(), powerController.getOutput(currentPosition, target));

            //simple P controller to correct if the robot isn't holding angle properly on its own
            //the plain decimal is the P constant. A value of 0.1 would give a power of 1 when robot is 10 degrees off heading
            //too low of a value and it won't correct enough, too high and the robot starts to oscillate
            double rotationSpeed = 0.08 * (robot.gyroscope.getAngle(AngleUnit.DEGREES) - initialHeading);

            robot.drivetrain.setPower(translationPower, robotCentricAngle, 0, AngleUnit.RADIANS);

        }

        //stop after movement is complete
        robot.drivetrain.setPower(0, theta, 0, AngleUnit.RADIANS);
    }



    //here because it has to be
    @Override
    public void runOpMode(){}
}
