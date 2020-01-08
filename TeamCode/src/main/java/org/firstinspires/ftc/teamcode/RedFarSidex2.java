package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoBackend.CustomSkystoneDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Red Far Side")
public class RedFarSidex2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public double robotSpeed = 0.45;
    OmegaBot robot;
    OmegaPID drivePID;
    MotionMethods motionMethods;
    private OpenCvCamera phoneCam;
    private CustomSkystoneDetector skyStoneDetector;

    String skystonePosition = "none";
    double xPosition;
    double yPosition;

    public void runOpMode() {
        robot = new OmegaBot(telemetry, hardwareMap);
        drivePID = robot.drivePID;
        motionMethods = new MotionMethods(robot, telemetry, this);
        /*
         */
        robot.drivetrain.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        skyStoneDetector = new CustomSkystoneDetector();
        skyStoneDetector.useDefaults();
        phoneCam.setPipeline(skyStoneDetector);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        // number of inches the robot strafes back depending
        // on the position of the skystone closest to the Skybridge
        double front= 0;
        double back = 0;
        double intakeAngle = 0;
        //All comments comment above what is being commented

        while (!isStopRequested() && !opModeIsActive()) {
            xPosition = skyStoneDetector.foundRectangle().x;
            yPosition = skyStoneDetector.foundRectangle().y;

            if (xPosition >= 75) { //TODO Tune these numbers
                skystonePosition = "right";
                back = 0;
                front = 0;
                intakeAngle = 65;
            } else if (xPosition > 10) {//x = 12
                skystonePosition = "center";
                back = 16;
                front = 4;
                intakeAngle = 110;
            } else {
                skystonePosition = "left";
                back = 11;
                front = 8;
                intakeAngle = 110;
            }

            telemetry.addData("xPos", xPosition);
            telemetry.addData("yPos", yPosition);
            telemetry.addData("SkyStone Pos", skystonePosition);
            telemetry.update();
        }
        waitForStart();

        // set initial values for arm, block gripper, pivot, and intakes
        robot.arm.setTargetPosition(-400);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.5);
        robot.blockGripper.setPosition(.75);
        robot.pivot.setPosition(.62);
        sleep(500);
        robot.leftIntake.setPower(-1);
        robot.rightIntake.setPower(1);


        motionMethods.strafe(180, .2, 1);

        //turns to angle and moves
        motionMethods.moveMotionProfile(back, 1);
        motionMethods.turnUsingPIDVoltageFieldCentric(intakeAngle, .5);
        motionMethods.moveMotionProfile(28, 1);

        robot.arm.setTargetPosition(10);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.5);
        robot.blockGripper.setPosition(0.1);
        robot.leftIntake.setPower(0);
        robot.rightIntake.setPower(0);
        sleep(500);
        robot.arm.setTargetPosition(-150);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.5);

        //faces parallel to red wall and moves to building site
        motionMethods.turnUsingPIDVoltageFieldCentric(-90,1);
        motionMethods.moveMotionProfile(10, 1);

        //corrects angle before going to building site
        motionMethods.turnUsingPIDVoltageFieldCentric(0, 1);
        motionMethods.moveMotionProfile(front, 1);
        motionMethods.moveMotionProfile(55,1);

        //turns and picks up foundation
        motionMethods.turnUsingPIDVoltageFieldCentric(-90, 1);
        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfile(12,1);
        robot.drivetrain.reverseDirection();

        robot.centerGripper.setPosition(1.00);
        //arm moves down and drops the brick
        robot.arm.setTargetPosition(-1700);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.5);
        sleep(500);
        //let go of brick
        robot.blockGripper.setPosition(.75);
        sleep(750);
        robot.arm.setTargetPosition(-200);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.5);
        motionMethods.moveMotionProfile(32,1);

        //turns and places into area
        motionMethods.turnUsingPIDVoltageFieldCentric(-180, .25);
        robot.centerGripper.setPosition(.51);
        robot.drivetrain.reverseDirection();
        motionMethods.moveMotionProfile(8, 1);
        robot.drivetrain.reverseDirection();
        motionMethods.turnUsingPIDVoltageFieldCentric(-180, 1);
        motionMethods.moveMotionProfile(20, 1);


    }
}