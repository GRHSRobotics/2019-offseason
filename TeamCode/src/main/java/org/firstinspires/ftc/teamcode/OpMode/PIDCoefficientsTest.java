package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.math.PIDController;

@TeleOp(name="PIDCoefficientsTest", group="PID")
public class PIDCoefficientsTest extends LinearOpMode {

    Robot robot;
    ElapsedTime timer;

    double targetPosition = 30; //inches
    double currentPosition = 0; //inches
    double power;

    public void runOpMode(){

        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer = new ElapsedTime();

        PIDController controller = new PIDController(0.8, 0, 10, timer, telemetry);
        controller.setMaxI(0.3);

        waitForStart();
/*
        while(opModeIsActive()){
            currentPosition = robot.drivetrain.right.getCurrentPosition() / robot.drivetrain.COUNTS_PER_INCH; //converts ticks to inches


            power = Math.min(controller.getOutput(currentPosition, targetPosition), 1); //converts inches to ticks


            robot.drivetrain.left.setPower(power);
            robot.drivetrain.right.setPower(power);

            telemetry.addData("Current Position, ", currentPosition);
            telemetry.addData("Power: ", power);
            telemetry.addData("Time (s):", timer.seconds());
            telemetry.update();

        }
*/

    }
}
