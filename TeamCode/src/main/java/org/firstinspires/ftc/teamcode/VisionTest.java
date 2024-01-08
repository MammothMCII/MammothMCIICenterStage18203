package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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


    @Override
    public void runOpMode() throws InterruptedException {

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

        Pose2d startPose = new Pose2d(-39, -60, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        //build trajectories

        Trajectory Red_Forward = drive.trajectoryBuilder(startPose)
                .forward(26)
                .build();
        Trajectory RedL_Left = drive.trajectoryBuilder(Red_Forward.end())
                .strafeLeft(8)
                .build();
        Trajectory RedL_Return = drive.trajectoryBuilder(RedL_Left.end())
                .strafeRight(100)
                .build();

        Trajectory RedR_Right = drive.trajectoryBuilder(Red_Forward.end())
                .strafeRight(16.5)
                .build();
        Trajectory RedR_Return = drive.trajectoryBuilder(RedR_Right.end())
                .strafeRight(85)
                .build();

        Trajectory RedC_Forward = drive.trajectoryBuilder(startPose)
                .forward(31)
                .build();
        Trajectory RedC_Back = drive.trajectoryBuilder(RedC_Forward.end())
                .back(5)
                .build();
        Trajectory RedC_Return = drive.trajectoryBuilder(RedC_Back.end())
                .strafeRight(90)
                .build();
        
        Trajectory RedL_To_Tape = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-31, -34))
                .build();
        Trajectory RedL_ReturnL = drive.trajectoryBuilder(RedL_To_Tape.end())
                .lineTo(new Vector2d(-31, 70))  //change the Y value to something more correct later
                .build();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values B", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Values R", valLeftR+"   "+valMidR+"   "+valRightR);




            telemetry.update();
            sleep(100);

            if (valLeft != -1 || valRight != -1 || valMid != -1 || valLeftR != -1 || valRightR != -1 || valMidR != -1 || valLeft != 0 || valRight != 0 || valMid != 0 || valLeftR != 0 || valRightR != 0 || valMidR != 0) {
                telemetry.addData("Values B", valLeft+"   "+valMid+"   "+valRight);
                telemetry.addData("Values R", valLeftR+"   "+valMidR+"   "+valRightR);
                telemetry.update();

                if(isStopRequested()) return;

                if (valLeft != 0) {

                }
                if (valRight != 0) {
                    //Blue Right
                }
                if (valMid != 0) {
                    //Blue mid
                }


                if (valLeftR != 0) {
                    drive.followTrajectory(RedL_To_Tape);
                    /* 
                    drive.followTrajectory(Red_Forward);
                    drive.followTrajectory(RedL_Left);
                    */
                    arm_tilt.setPower(1);
                    sleep(500);
                    arm_tilt.setPower(0);
                    drive.followTrajectory(RedL_ReturnL);
                    /* 
                    drive.followTrajectory(RedL_Return);
                    */
                }
                if (valRightR != 0) {
                    drive.followTrajectory(Red_Forward);
                    drive.followTrajectory(RedR_Right);
                    arm_tilt.setPower(1);
                    sleep(500);
                    arm_tilt.setPower(0);
                    drive.followTrajectory(RedR_Return);
                }
                if (valMidR != 0) {
                    drive.followTrajectory(RedC_Forward);
                    arm_tilt.setPower(1);
                    sleep(500);
                    arm_tilt.setPower(0);
                    drive.followTrajectory(RedC_Back);
                    drive.followTrajectory(RedC_Return);
                }

                sleep(1233456);
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
            Core.extractChannel(yCbCrR, yCbCrR, 1);//takes ? difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 148, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(yCbCrR, thresholdR, 150, 255, Imgproc.THRESH_BINARY);

            //outline/contour dont know what this does
            //Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            double[] pixMidR = thresholdR.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMidR = (int)pixMidR[0];

            double[] pixLeftR = thresholdR.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeftR = (int)pixLeftR[0];

            double[] pixRightR = thresholdR.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRightR = (int)pixRightR[0];



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
}