package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="pushwaffle")
public class PushWaffle extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public double robotSpeed = 0.45;
    OmegaBot robot;
    OmegaPID drivePID;
    MotionMethods motionMethods;

    public void runOpMode() {
        robot = new OmegaBot(telemetry, hardwareMap);
        drivePID = robot.drivePID;
        motionMethods = new MotionMethods(robot, telemetry, this);

        robot.drivetrain.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        robot.centerGripper.setPosition(.51);

        motionMethods.moveMotionProfile(5,1);
        motionMethods.turnUsingPIDVoltageFieldCentric(90,.5);

        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfile(11,1);
        robot.drivetrain.reverseDirection();

        motionMethods.turnUsingPIDVoltageFieldCentric(0,.5);
        motionMethods.moveMotionProfile(24,1);
        motionMethods.turnUsingPIDVoltageFieldCentric(0,.5);

        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfile(11,1);
        robot.drivetrain.reverseDirection();

        motionMethods.turnUsingPIDVoltageFieldCentric(0,.5);

        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfile(13,1);
        robot.drivetrain.reverseDirection();

        motionMethods.moveMotionProfile(3,1);
        motionMethods.turnUsingPIDVoltageFieldCentric(90,.5);
        motionMethods.moveMotionProfile(7,1);
        motionMethods.turnUsingPIDVoltageFieldCentric(120,.5);
        motionMethods.moveMotionProfile(8,1);
    }
}