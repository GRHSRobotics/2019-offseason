package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;


@TeleOp(name="BasicTeleopTest", group="Basic")
public class BasicTeleopTest extends LinearOpMode {

    Robot robot;

    public void runOpMode(){

        robot = new Robot(hardwareMap, telemetry);
        robot.drivetrain.initIMU(hardwareMap);

        waitForStart(); //separates init period from start period

        while(opModeIsActive()){ //main opmode loop

            robot.drivetrain.setPowerRectangular(-gamepad1.left_stick_y, gamepad1.left_stick_x);

            telemetry.addData("Right", robot.drivetrain.right.getCurrentPosition());
            telemetry.addData("Left:", robot.drivetrain.left.getCurrentPosition());
            telemetry.addData("Front:", robot.drivetrain.front.getCurrentPosition());
            telemetry.addData("Back:", robot.drivetrain.back.getCurrentPosition());
            telemetry.update();



        }

    }
}
