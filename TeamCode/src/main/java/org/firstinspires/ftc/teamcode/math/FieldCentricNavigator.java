package org.firstinspires.ftc.teamcode.math;

public class FieldCentricNavigator {

    /*for the purpose of standardization, all angles used with this class should be in the range (-180, 180) or radian equivalent
    with 0 being forwards, negative being right, and positive being left.

    This class can be used for both robot translations and robot rotations.
     */

    double initialAngle; //the initial angle of the robot with respect to the field
    double desiredAngle; //the angle desired with respect to the field

    public FieldCentricNavigator(double initialAngle, double desiredAngle){
        this.initialAngle = initialAngle;
        this.desiredAngle = desiredAngle;
    }

    public void setDesiredAngle(double newDesiredAngle){
        desiredAngle = newDesiredAngle;
    }

    /**
     * Converts a field-centric angle into its corresponding robot-centric angle using rotation information about the robot
     * @param currentAngle the gyro reading AKA the current angle with respect to the robot's initial angle
     * @return the robot-centric angle necessary to achieve a given field-centric angle
     */
    public double fieldToRobotCentric(double currentAngle){
        return desiredAngle - currentAngle - initialAngle;
    }
}
