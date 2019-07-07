package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.math.PDController;

@TeleOp(name="PDCoefficientsTest", group="PD")
public class PDCoefficientsTest extends LinearOpMode {

    Robot robot;
    ElapsedTime timer;

    double targetPosition = 10; //inches
    double currentPosition = 0; //inches
    double error;
    double power;

    public void runOpMode(){

        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer = new ElapsedTime();

        PDController controller = new PDController(0.05, 0, timer);

        waitForStart();

        while(opModeIsActive()){
            currentPosition = robot.drivetrain.left.getCurrentPosition() * (3*Math.PI) / 1120; //converts ticks to inches


            power = controller.getOutput(currentPosition, targetPosition); //converts inches to ticks

            robot.drivetrain.left.setPower(0.4 * power);
            robot.drivetrain.right.setPower(power);

            telemetry.addData("Current Position, ", currentPosition);
            telemetry.addData("Power: ", power);
            telemetry.update();

        }


    }
}
