package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name="BasicAutonTest", group="Basic")
public class BasicAutonTest extends LinearOpMode {

    Robot robot;

    public void runOpMode(){

        robot = new Robot(hardwareMap, telemetry);

        waitForStart();

        robot.drivetrain.encoderDrive(10, 0, 1, 10);

    }
}
