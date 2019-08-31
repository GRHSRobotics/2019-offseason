package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name="FieldCentricTeleopTest", group="test")
public class FieldCentricTeleopTest extends LinearOpMode {

    Robot robot;

    public void runOpMode(){

        robot = new Robot(hardwareMap, telemetry);


    }
}
