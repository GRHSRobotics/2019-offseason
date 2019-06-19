package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

public class BasicTeleopTest extends LinearOpMode {

    Robot robot;

    public void runOpMode(){

        robot = new Robot(hardwareMap);

        waitForStart(); //separates init period from start period

        while(opModeIsActive()){ //main opmode loop

            robot.drivetrain.setPowerRectangular(-gamepad1.left_stick_y, gamepad1.left_stick_x);

        }

    }
}
