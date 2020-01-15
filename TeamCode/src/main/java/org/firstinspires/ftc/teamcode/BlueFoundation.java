package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Foundation")
public class BlueFoundation extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public double robotSpeed = 0.45;
    OmegaBot robot;
    OmegaPID drivePID;
    MotionMethods motionMethods;

    public void runOpMode() {
        robot = new OmegaBot(telemetry, hardwareMap);
        drivePID = robot.drivePID;
        motionMethods = new MotionMethods(robot, telemetry, this);
        /*
         */
        robot.drivetrain.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfile(33,1);
        robot.drivetrain.reverseDirection();
        motionMethods.turnUsingPIDVoltageFieldCentric(0, .5);
        robot.centerGripper.setPosition(1);
        motionMethods.moveMotionProfile(33,1);
        robot.centerGripper.setPosition(.51);
        motionMethods.turnUsingPIDVoltageFieldCentric(0, .5);
        motionMethods.strafe(0,1,1);

        //GYRO SETUP
        runtime.reset();
    }
}
