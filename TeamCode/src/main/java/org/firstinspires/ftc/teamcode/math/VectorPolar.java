package org.firstinspires.ftc.teamcode.math;

public class VectorPolar {

    private double magnitude;
    private double angle; //should always be in radians

    //constructor, makes sure class is created with the right data
    public VectorPolar (double magnitude, double angle, AngleUnit unit){
        this.magnitude = magnitude;

        if(unit == AngleUnit.RADIANS) {
            this.angle = angle;
        } else {
            this.angle = Math.toRadians(angle);
        }
    }

    public double getMagnitude(){
        return magnitude;
    }

    public void setMagnitude(double newMagnitude){
        magnitude = newMagnitude;
    }

    public double getAngle(AngleUnit unit){
        if (unit == AngleUnit.RADIANS) {
            return angle;
        } else {
            return Math.toDegrees(angle);
        }
    }

    public void setAngle(double newAngle, AngleUnit unit){
        if(unit == AngleUnit.RADIANS){
            angle = newAngle;
        } else{
            angle = Math.toRadians(newAngle);
        }

    }

    public double getX(){
        return magnitude * Math.cos(angle); //TODO: add sanity check go make sure that values that should be zero are actually zero
    }

    public double getY(){
        return magnitude * Math.sin(angle);
    }

}
