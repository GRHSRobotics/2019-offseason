package org.firstinspires.ftc.teamcode.math;

public class VectorRectangular extends Vector {

    private double y;
    private double x;

    public VectorRectangular (double y, double x){
        this.y = y;
        this.x = x;
    }

    public void setX(double newX){
        x = newX;
    }

    public void setY(double newY){
        y = newY;
    }

    /**
     *
     * @param unit angle unit
     * @return angle of the vector in a range from (-pi, pi) radians or (-180, 180) degrees
     */
    public double getAngle(angleUnit unit){
        if(unit == angleUnit.RADIANS){
            return Math.atan2(y, x);
        } else {
            return Math.toDegrees(Math.atan2(y, x));
        }
    }


    /**\
     *
     * @return the magnitude of the vector
     */
    public double getMagnitude(){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }



}
