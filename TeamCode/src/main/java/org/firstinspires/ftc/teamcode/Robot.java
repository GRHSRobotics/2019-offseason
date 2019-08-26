package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Gyroscope;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystem.XDrivetrain;


//This class contains all robot hardware organized by subsystem
//see subsystem specific classes for hardware naming requirements
public class Robot {

    public MecanumDrivetrain drivetrain; //putting this here is so that the scope is right
    public Gyroscope gyroscope;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry){

        drivetrain = new MecanumDrivetrain(hardwareMap, telemetry);
        gyroscope = new Gyroscope(hardwareMap, telemetry);


    }

}
