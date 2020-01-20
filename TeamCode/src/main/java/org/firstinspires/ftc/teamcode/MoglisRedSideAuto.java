package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoBackend.CustomSkystoneDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "MoglisRedSideAuto")
        public class MoglisRedSideAuto extends LinearOpMode {
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
                double front = 0;
                double back = 0;
                double intakeAngle = 0;
                //All comments comment above what is being commented

                while (!isStopRequested() && !opModeIsActive()) {
                    xPosition = skyStoneDetector.foundRectangle().x;
                    yPosition = skyStoneDetector.foundRectangle().y;

                    if (xPosition >= 75) { //TODO Tune these numbers
                        skystonePosition = "right";
                        back = 14;
                    } else if (xPosition > 17) {//x = 12
                        skystonePosition = "center";
                        back = 8;
                    } else {
                        skystonePosition = "left";
                        back = 4;
                    }

                    telemetry.addData("xPos", xPosition);
                    telemetry.addData("yPos", yPosition);
                    telemetry.addData("SkyStone Pos", skystonePosition);
                    telemetry.update();
                }
                waitForStart();

                //Initialization for teleop/side gripper
                robot.cap.setPosition(1);
                //gripper to super open?
                robot.rightGripper.setPosition(0.1);
                //elbow down
                robot.arm.setTargetPosition(-200);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(.5);
                //teleop arm up


                //gets in position and strafes to skystone
                motionMethods.moveMotionProfile(3.1, 1);
                //change the distance to back once tuned
                motionMethods.strafe(180, .78, 1);
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                sleep(100);
                //double time = runtime.milliseconds();
                //while(runtime.milliseconds()<time+700);
                //intakes stone
                robot.rightGripper.setPosition(0);
                //elbow down
                sleep(400);
                robot.cap.setPosition(.45);
                //gripper closed
                sleep(500);
                robot.rightGripper.setPosition(.37);
                //elbow up
                sleep(800);

                //moving to the other side
                motionMethods.strafe(0, .28, 0.5);
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                motionMethods.moveMotionProfile(65,1);
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                //go across the field to deposit in foundation
                motionMethods.strafe(180, .01, 0.1);

                //dump on foundation
                sleep(800);
                robot.rightGripper.setPosition(0);
                //elbow down
                robot.cap.setPosition(1);
                //gripper to super open?
                robot.rightGripper.setPosition(0.37);
                //elbow up

                //move to position to intake 2nd skystone
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                motionMethods.strafe(0, .28, 0.5);
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                sleep(400);
                robot.drivetrain.reverseDirection();
                motionMethods.moveMotionProfileReverse(78,1);
                //change distance for 2nd skystone
                robot.drivetrain.reverseDirection();
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );

                //final line up/intake of 2nd skystone
                robot.rightGripper.setPosition(0.1);
                motionMethods.strafe(180, .5, 0.01);
                sleep(800);
                robot.cap.setPosition(.45);
                //gripper closed
                sleep(800);
                robot.rightGripper.setPosition(.37);
                //elbow up
                sleep(800);

                //move to deposit on foundation
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                motionMethods.strafe(0, .01, 0.1);
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1 );
                motionMethods.moveMotionProfile(96,1);
                //change distance experimentally

                //strafes and intakes stone
                motionMethods.strafe(180, .01, 0.1);
                motionMethods.turnUsingPIDVoltageFieldCentric(0,1);
                sleep(100);
                //intakes stone
                robot.rightGripper.setPosition(0);
                //elbow down
                sleep(400);
                robot.cap.setPosition(.45);
                //gripper closed
                sleep(500);
                robot.rightGripper.setPosition(.37);
                //elbow up
                sleep(800);

                //strafes and moves to foundation
                motionMethods.strafe(0,.28, 1);
                motionMethods.moveMotionProfile(90,1);


            }
}

