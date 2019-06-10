package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;

public class XDrivetrain {

    public DcMotor front;
    public DcMotor back;
    public DcMotor left;
    public DcMotor right;

    public XDrivetrain(DcMotor front, DcMotor back, DcMotor left, DcMotor right){
        this.front = front;
        this.back = back;
        this.left = left;
        this.right = right;

        //change motor directions so that positive powers move the right way
        this.front.setDirection(DcMotor.Direction.FORWARD);
        this.back.setDirection(DcMotor.Direction.REVERSE);
        this.left.setDirection(DcMotor.Direction.FORWARD);
        this.right.setDirection(DcMotor.Direction.REVERSE);

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
     *
     * @param forwardPower power in the y direction in the range [-1,1]
     * @param strafePower power in the x direction in the range [-1,1]
     */
    public void setPower(double forwardPower, double strafePower){
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
}
