package org.firstinspires.ftc.teamcode.math;

public class VectorRectangular {

    private double y;
    private double x;

    public VectorRectangular (double y, double x){
        this.y = y;
        this.x = x;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
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
    public double getAngle(AngleUnit unit){

        double output = Math.atan2(y, x); //all java trig functions in radians
        if(output < 0){
            output += 2 * Math.PI;
        }

        if(unit == AngleUnit.RADIANS){
            return output;
        } else {
            return Math.toDegrees(output);
        }
    }


    /**
     *
     * @return the magnitude of the vector
     */
    public double getMagnitude(){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }


}
