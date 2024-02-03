package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.stat.descriptive.rank.Max;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Point;

import java.util.Random;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;

import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.AutonomousConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name= "VisionFront", group="autonomous")
//@Disabled//comment out this line before using
public class VisionTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static int valMidR = -1;
    private static int valLeftR = -1;
    private static int valRightR = -1;

    private static int max = 0;


    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4.1f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {0.5f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {7.5f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 1280;
    private final int cols = 720;

    OpenCvCamera phoneCam;


    private DcMotor arm_tilt;

    private Servo top_grip;
    private Servo hand_tilt;
    private Servo bottom_grip;
    private DcMotor arm_slide;


    @Override
    public void runOpMode() throws InterruptedException {

        top_grip = hardwareMap.get(Servo.class, "top_grip");
        bottom_grip = hardwareMap.get(Servo.class, "bottom_grip");
        hand_tilt = hardwareMap.get(Servo.class, "hand_tilt");
        arm_tilt = hardwareMap.get(DcMotor.class, "arm_tilt");
        arm_slide = hardwareMap.get(DcMotor.class, "arm_slide");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        arm_tilt = hardwareMap.get(DcMotor.class, "arm_tilt");

        //roadrunner initialization
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPoseRed = new Pose2d(-39, -60, Math.toRadians(90));

        Pose2d startPoseBlue = new Pose2d(-30, 60, Math.toRadians(-90));



        //build trajectories



        // red L
        Trajectory RedL_To_Tape = drive.trajectoryBuilder(startPoseRed)
                .lineToConstantHeading(new Vector2d(-48, -38))
                .build();
        Trajectory RedL_ReturnL = drive.trajectoryBuilder(RedL_To_Tape.end())
                .lineToConstantHeading(new Vector2d(-48, -40))
                .splineToConstantHeading(new Vector2d(-60, -40), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-60, -24), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-50, -9, Math.toRadians(0)), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(53, -9))
                .build();

        // red M
        Trajectory RedM_To_Tape = drive.trajectoryBuilder(startPoseRed)
                .lineToConstantHeading(new Vector2d(-39, -29))
                .build();

        Trajectory RedM_Return = drive.trajectoryBuilder(RedM_To_Tape.end())
                .splineToConstantHeading(new Vector2d(-39, -33), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(53, -33))
                .build();

        // red R
        Trajectory RedR_To_Tape = drive.trajectoryBuilder(startPoseRed)
                .splineToConstantHeading(new Vector2d(-24, -32), Math.toRadians(0))
                .build();

        Trajectory RedR_Return = drive.trajectoryBuilder(RedR_To_Tape.end())
                .lineToConstantHeading(new Vector2d(-26, -33))
                .splineToConstantHeading(new Vector2d(-60, -33), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-60, -24), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-50, -9, Math.toRadians(0)), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(53, -9))
                .build();


        // Blue R
        Trajectory BlueR_To_Tape = drive.trajectoryBuilder(startPoseBlue)
                .lineToConstantHeading(new Vector2d(-47, 33))
                .build();
        Trajectory BlueR_Return = drive.trajectoryBuilder(BlueR_To_Tape.end())
                .lineToConstantHeading(new Vector2d(53, 33))
                .build();

        // Blue M
        Trajectory BlueM_To_Tape = drive.trajectoryBuilder(startPoseBlue)
                .lineToConstantHeading(new Vector2d(-39, 29))
                .build();
        Trajectory BlueM_Reverse = drive.trajectoryBuilder(BlueM_To_Tape.end())
                .lineToConstantHeading(new Vector2d(-39, 33))
                .build();
        Trajectory BlueM_Return = drive.trajectoryBuilder(BlueM_Reverse.end())
                .lineToConstantHeading(new Vector2d(53, 33))
                .build();

        // Blue L
        Trajectory BlueL_To_Tape = drive.trajectoryBuilder(startPoseBlue)
                .splineToConstantHeading(new Vector2d(-22, 33), Math.toRadians(0))
                .build();

        Trajectory BlueL_Return = drive.trajectoryBuilder(BlueL_To_Tape.end())
                .lineToConstantHeading(new Vector2d(53, 33))
                .build();

        bottom_grip.setPosition(0);
        top_grip.setPosition(1);

        telemetry.addData("Robot has initialized", ")");
        telemetry.update();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values B", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Values R", valLeftR+"   "+valMidR+"   "+valRightR);

            telemetry.update();

            while (valLeft != -1 || valRight != -1 || valMid != -1 || valLeftR != -1 || valRightR != -1 || valMidR != -1) {
                telemetry.addData("Values B", valLeft+"   "+valMid+"   "+valRight);
                telemetry.addData("Values R", valLeftR+"   "+valMidR+"   "+valRightR);
                telemetry.addData("max", max);
                telemetry.update();
                hand_tilt.setPosition(0.48);

                if(isStopRequested()) return;


                if (valLeft == max || valMid == max || valRight == max) {
                    drive.setPoseEstimate(startPoseBlue);
                    if (valLeft == max) {
                        //blue left
                        drive.followTrajectory(BlueL_To_Tape);
                        dropPixel();
                        drive.followTrajectory(BlueL_Return);
                        top_grip.setPosition(0);
                        sleep(1233456);
                    }
                    if (valRight == max) {
                        //Blue Right
                        drive.followTrajectory(BlueR_To_Tape);
                        dropPixel();
                        drive.followTrajectory(BlueR_Return);
                        top_grip.setPosition(0);
                        sleep(1233456);
                    }
                    if (valMid == max) {
                        //Blue mid
                        drive.followTrajectory(BlueM_To_Tape);
                        dropPixel();
                        drive.followTrajectory(BlueM_Reverse);
                        drive.followTrajectory(BlueM_Return);
                        top_grip.setPosition(0);
                        sleep(1233456);
                    }
                }
                if (valLeftR == max || valMidR == max || valRightR == max) {
                    drive.setPoseEstimate(startPoseRed);
                    if (valLeftR == max) {
                        drive.followTrajectory(RedL_To_Tape);
                        dropPixel();
                        drive.followTrajectory(RedL_ReturnL);
                        top_grip.setPosition(0);
                        sleep(1233456); // without this sleep the robot will follow an extra trajectory, dont know why
                    }
                    if (valRightR == max) {
                        drive.followTrajectory(RedR_To_Tape);
                        dropPixel();
                        drive.followTrajectory(RedR_Return);
                        top_grip.setPosition(0);
                        sleep(1233456);
                    }
                    if (valMidR == max) {
                        drive.followTrajectory(RedM_To_Tape);
                        dropPixel();
                        hand_tilt.setPosition(0.2);
                        drive.followTrajectory(RedM_Return);
                        top_grip.setPosition(0);
                        sleep(1233456);
                    }
                }


            }

        }
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat yCbCrR = new Mat();
        Mat thresholdMat = new Mat();
        Mat thresholdR = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            Imgproc.cvtColor(input, yCbCrR, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrR, yCbCrR, 1);//takes cr difference and stores

            yCbCrChan2Mat.copyTo(all);//copies mat object



            double[] pixMid = yCbCrChan2Mat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = yCbCrChan2Mat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = yCbCrChan2Mat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            double[] pixMidR = yCbCrR.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMidR = (int)pixMidR[0];

            double[] pixLeftR = yCbCrR.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeftR = (int)pixLeftR[0];

            double[] pixRightR = yCbCrR.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRightR = (int)pixRightR[0];


            max = Math.max(valMid, Math.max(valLeft, Math.max(valRight, Math.max(valLeftR, Math.max(valMidR, valRightR)))));




            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
    public void dropPixel(){
        bottom_grip.setPosition(1);
        //top_grip.setPosition(0);
        sleep(200);
        arm_slide.setPower(1);
        arm_tilt.setPower(1);
        sleep(100);
        arm_slide.setPower(0);
        arm_tilt.setPower(0);
        //hand_tilt.setPosition(0.2);

    }
}