package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "LinearTeleop", group = "prototype")
public class LinearTeleop extends LinearOpMode {
    private OmegaBot robot;
    private ElapsedTime runtime;
    double maxSpeed = 1;
    boolean intakeOn = true;
    int armPos = 0;

    public void runOpMode(){
        robot = new OmegaBot(telemetry,hardwareMap);//initializing hardware
        waitForStart();
        runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        robot.leftIntake.setPower(1);
        robot.rightIntake.setPower(-1);
        while(opModeIsActive()){
            drivetrainProcess();
            armProcess();
            servoProcess();
            intakeProcessIn();
            intakeProcessOut();
            foundationGrippers();
            telemetry.addData("armPos",robot.arm.getCurrentPosition());
            telemetry.update();
        }
    }

    public void moveArm(double degrees){
        double maxVel = 15 * 360 / 60000; // 15 is the rotations per minute, 360 is the degrees per rotation, 60000 is the number of milliseconds in a minute
        double macAcc = maxVel / 1300; //1300 is the number of milliseconds it takes to accelerate to full speed
        MotionProfileGenerator generator = new MotionProfileGenerator(maxVel, macAcc);
        double[] motionProfile = generator.generateProfile(degrees);
        double[] distanceProfile = generator.generateDistanceProfile(motionProfile);
        runtime.reset();
        while(runtime.milliseconds() < motionProfile.length && opModeIsActive()){
            robot.arm.setPower(motionProfile[(int)runtime.milliseconds()]/maxVel);//TODO: use the distance profile + encoders to pid up in dis bicth
        }
    }

    public void servoProcess(){
        if(gamepad2.x){
            robot.pivot.setPosition(0.62);
        }else if(gamepad2.y){
            robot.pivot.setPosition(.96);
        }
        if(gamepad2.right_bumper){
            robot.blockGripper.setPosition(.75);
        }else if(gamepad2.left_bumper){
            robot.blockGripper.setPosition(.3);
        }
    }

    public void foundationGrippers(){
        if(gamepad1.left_trigger > 0.5){
            robot.centerGripper.setPosition(1.00);
        }
        else if (gamepad1.right_trigger > 0.5){
            robot.centerGripper.setPosition(0.51);
        }
    }

    public void armProcess(){
        if(gamepad2.a){
            armPos -= 5;
            //robot.arm.setPower(.75);
            //moveArm(-30);
        }
        else if(gamepad2.b){
            armPos += 5;
            //moveArm(30);
            //robot.arm.setPower(-.75);
        }//else{
            //robot.arm.setPower(0);
        //}
        robot.arm.setTargetPosition(armPos);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(0.5);
    }

    public void intakeProcessIn(){
        if(gamepad2.right_trigger > .5) {
            if (gamepad2.left_trigger > .5 && gamepad2.right_trigger > .5){
                robot.leftIntake.setPower(0);
                robot.rightIntake.setPower(0);
            }
            else {
                robot.leftIntake.setPower(1);
                robot.rightIntake.setPower(-1);
            }

        }

        else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }
    }
    public void intakeProcessOut(){
        if (gamepad2.left_trigger > .5){
            if (gamepad2.left_trigger > .5 && gamepad2.right_trigger > .5){
                robot.leftIntake.setPower(0);
                robot.rightIntake.setPower(0);
            }
            else {
                robot.leftIntake.setPower(-1);
                robot.rightIntake.setPower(1);
            }

        }

        else {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }

    }

    public void drivetrainProcess(){
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double clockwise = gamepad1.right_stick_x *.75;
        //double temp = forward * Math.cos(Math.toRadians(robot.getAngle())) - right * Math.sin(Math.toRadians(robot.getAngle()));
        //right = forward * Math.sin(Math.toRadians(robot.getAngle())) + right * Math.cos(Math.toRadians(robot.getAngle()));
        //forward = temp;

        double front_left = forward + clockwise + right;
        double front_right = forward - clockwise - right;
        double rear_left = forward + clockwise - right;
        double rear_right = forward - clockwise + right;

        //double FrontLeftVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
        //double FrontRightVal = gamepad1.left_stick_y + (gamepad1.left_stick_x) - -gamepad1.right_stick_x;
        //double BackLeftVal = gamepad1.left_stick_y + (gamepad1.left_stick_x) + -gamepad1.right_stick_x;
        //double BackRightVal = gamepad1.left_stick_y - (gamepad1.left_stick_x) - -gamepad1.right_stick_x;

        double max = Math.abs(front_left);
        if (Math.abs(front_right) > max) max = Math.abs(front_right);
        if (Math.abs(rear_left) > max) max = Math.abs(rear_left);
        if (Math.abs(rear_right) > max) max = Math.abs(rear_right);

        if (max > maxSpeed) {
            front_left /= (max/maxSpeed);
            front_right /= (max/maxSpeed);
            rear_left /= (max/maxSpeed);
            rear_right /= (max/maxSpeed);
        }

        robot.frontLeft.setPower(front_left);
        robot.frontRight.setPower(front_right);
        robot.backLeft.setPower(rear_left);
        robot.backRight.setPower(rear_right);
    }
}
