package org.firstinspires.ftc.teamcode;
package com.acmerobotics.roadrunner.drive;
package com.acmerobotics.roadrunner.path;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//roadrunner imports
import com.acmerobotics.roadrunner.*;

@Autonomous(name = "StraightTest")
//@Disabled
public class StraightTest extends LinearOpMode {
    OmegaBot robot;
    MotionMethods motionMethods;
    ;

    public void runOpMode(){
        robot = new OmegaBot(telemetry, hardwareMap);
        motionMethods = new MotionMethods(robot, telemetry, this);

        robot.drivetrain.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        robot.arm.setTargetPosition(-400);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // specify coefficients/gains
        PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
        // create the controller
        PIDFController controller = new PIDFController(coeffs);

        // specify the setpoint
        controller.setTargetPosition(setpoint);
        double correction = controller.update(measuredPosition);
        PIDFController controller = new PIDFController(coeffs, 0, 0, 0, x -> kG);


    }
}
