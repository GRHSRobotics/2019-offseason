package org.firstinspires.ftc.teamcode.math;

public class VectorPolar extends Vector {

    private double magnitude;
    private double angle; //should always be in radians

    //constructor, makes sure class is created with the right data
    public VectorPolar (double magnitude, double angle, angleUnit unit){
        this.magnitude = magnitude;

        if(unit == angleUnit.RADIANS) {
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

    public double getAngle(angleUnit unit){
        if (unit == angleUnit.RADIANS) {
            return angle;
        } else {
            return Math.toDegrees(angle);
        }
    }

    public void setAngle(double newAngle, angleUnit unit){
        if(unit == angleUnit.RADIANS){
            angle = newAngle;
        } else{
            angle = Math.toRadians(newAngle);
        }

    }

    public double getX(){
        return magnitude * Math.cos(angle);
    }

    public double getY(){
        return magnitude * Math.sin(angle);
    }
}
