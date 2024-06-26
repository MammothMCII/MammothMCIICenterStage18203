package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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


import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name= "VisionBack", group="autonomous")
@Disabled

public class VisionBack extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private ElapsedTime wait_stop = new ElapsedTime();


    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static int valMidR = -1;
    private static int valLeftR = -1;
    private static int valRightR = -1;

    private static int max = 0;

    private static int wait_time = 0;

    private static int end_pos = 0; //0 = left,   1 = right,   2 = middle

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
    private DcMotor topleftmotor;
    private DcMotor bottomleftmotor;
    private DcMotor toprightmotor;
    private DcMotor bottomrightmotor;


    enum pos{
        left,
        middle,
        right
    }


    @Override
    public void runOpMode() throws InterruptedException {

        topleftmotor = hardwareMap.get(DcMotor.class, "top left motor");
        bottomleftmotor = hardwareMap.get(DcMotor.class, "bottom left motor");
        toprightmotor = hardwareMap.get(DcMotor.class, "top right motor");
        bottomrightmotor = hardwareMap.get(DcMotor.class, "bottom right motor");

        topleftmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomleftmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        toprightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        top_grip = hardwareMap.get(Servo.class, "top_grip");
        bottom_grip = hardwareMap.get(Servo.class, "bottom_grip");
        hand_tilt = hardwareMap.get(Servo.class, "hand_tilt");
        arm_tilt = hardwareMap.get(DcMotor.class, "arm_tilt");
        arm_slide = hardwareMap.get(DcMotor.class, "arm_slide");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        arm_tilt = hardwareMap.get(DcMotor.class, "arm_tilt");

        //roadrunner initialization


        Pose2d startPoseRed = new Pose2d(9, -60, Math.toRadians(90));

        Pose2d startPoseBlue = new Pose2d(17, 60, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPoseRed);


        //build trajectories --------------------------------------------------------

        // red to tape
        Action RedL_To_Tape = drive.actionBuilder(startPoseRed)
                .splineToConstantHeading(new Vector2d(0, -31), Math.toRadians(180))
                .build();

        Action RedM_To_Tape = drive.actionBuilder(startPoseRed)
                .lineToXConstantHeading(12)
                .lineToYConstantHeading(-29)
                .build();

        Action RedR_To_Tape = drive.actionBuilder(startPoseRed)
                .splineToConstantHeading(new Vector2d(25, -34), Math.toRadians(0))
                .build();


        // red to wait pos
        Action Red_to_waitR = drive.actionBuilder(drive.pose)
                //.splineToSplineHeading(new Pose2d(30, -45, Math.toRadians(0)), Math.toRadians(0))
                .lineToXLinearHeading(30, Math.toRadians(0))
                .lineToYLinearHeading(-45, Math.toRadians(0))
                //.splineToSplineHeading(new Pose2d(58, -14, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Action Red_to_wait = drive.actionBuilder(drive.pose)
                .lineToXConstantHeading(1)
                .lineToYConstantHeading( -32.5)
                .splineToConstantHeading(new Vector2d(20, -33), Math.toRadians(0))
                //.splineToSplineHeading(new Pose2d(58, -13, Math.toRadians(0)), Math.toRadians(0))
                .build();

        //red return
        Action Red_Return = drive.actionBuilder(drive.pose)
                //.lineToConstantHeading(new Vector2d(30, -9))
                .splineToLinearHeading(new Pose2d(40, -25, Math.toRadians(0)), 0)
                .build();

        //red board
        Action Place_On_board_RedL = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(56, -27), 0)
                .build();

        Action Place_On_board_RedM = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(56, -35), 0)
                .build();

        Action Place_On_board_RedR = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(56, -42), 0)
                .build();


        //park
        Action Red_Place_returnL = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(35, -10), 0)
                .splineToConstantHeading(new Vector2d(53, -7), 0)
                .build();

        Action Red_Place_returnR = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(35, -40), 0)
                .splineToConstantHeading(new Vector2d(53, -60), 0)
                .build();



        ///--------------------------------------------------------------------------------------------------
        // Blue R to tape
        Action BlueR_To_Tape = drive.actionBuilder(startPoseBlue)
                .splineToConstantHeading(new Vector2d(1, 32), Math.toRadians(180))
                .build();

        Action BlueL_To_Tape = drive.actionBuilder(startPoseBlue)
                .splineToConstantHeading(new Vector2d(26, 33), Math.toRadians(0))
                .build();

        Action BlueM_To_Tape = drive.actionBuilder(startPoseBlue)
                .splineToConstantHeading(new Vector2d(12, 28), 0)
                .build();



        // Blue to wait pos
        Action Blue_to_waitL = drive.actionBuilder(drive.pose)
                //.lineToConstantHeading(new Vector2d(30, 45))
                //.splineToSplineHeading(new Pose2d(58, 13, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(30, 45, Math.toRadians(0)), 0)
                .build();

        Action Blue_to_wait = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(1, 32.5), 0)
                .splineToConstantHeading(new Vector2d(20, 32.5), Math.toRadians(0))

                //.splineToSplineHeading(new Pose2d(58, 13, Math.toRadians(0)), Math.toRadians(0))
                .build();

        //Blue return
        Action Blue_Return = drive.actionBuilder(drive.pose)
                //.lineToConstantHeading(new Vector2d(30, 9))
                //.splineToConstantHeading(new Vector2d(40, 20), 0)

                .splineToLinearHeading(new Pose2d(40, 25, Math.toRadians(0)), 0)
                .build();


        //Blue board
        Action Place_On_board_BlueR = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(55, 24.5), 0)
                .build();

        Action Place_On_board_BlueM = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(55, 33), 0)
                .build();

        Action Place_On_board_BlueL = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(55, 38), 0)
                .build();


        //park
        Action Blue_Place_returnL = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(39, 40), 0)
                .splineToConstantHeading(new Vector2d(53, 60), 0)
                .build();

        Action Blue_Place_returnR = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(35, 10), 0)
                .splineToConstantHeading(new Vector2d(53, 7), 0)
                .build();

        ///----------------------------------------------------------------------------------------------------

        bottom_grip.setPosition(0);
        top_grip.setPosition(0.8);

        String middle = "Middle";
        String left = "Left";
        String right = "Right";
        String three = "3";
        String no = "no";
        String five = "5";
        String tmax = "max";

        //telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));   #ref
        while(true){
            telemetry.addLine(String.format("░░░░░■%s░┌---------┐░░░▲:%s░░░░░", middle, three));
            telemetry.addLine(String.format("░%s■=╩=■%s|  accept |░■:%s░░O:%s░", left, right, no, five));
            telemetry.addLine(String.format("░░░░░░░░░░░└---------┘░░░░X:%s░░", tmax));
            telemetry.addLine("DO NOT HIT START");

            telemetry.update();
            if (gamepad1.x){
                wait_time = 0;
                three = "3";
                no = "n\u0332o\u0332";
                five = "5";
                tmax = "max";
            }
            else if (gamepad1.y){
                wait_time = 3;
                three = "3\u0332";
                no = "no";
                five = "5";
                tmax = "max";
            }
            else if (gamepad1.a){
                wait_time = 7;
                three = "3";
                no = "no";
                five = "5";
                tmax = "m\u0332a\u0332x\u0332";
            }
            else if (gamepad1.b){
                wait_time = 5;
                three = "3";
                no = "no";
                five = "5\u0332";
                tmax = "max";
            }
            if (gamepad1.dpad_left){
                end_pos = 0;
                middle = "Middle";
                left = "L\u0332e\u0332f\u0332t\u0332";
                right = "Right";
            }
            else if (gamepad1.dpad_up){
                end_pos = 2;
                middle = "M\u0332i\u0332d\u0332d\u0332l\u0332e\u0332";
                left = "Left";
                right = "Right";
            }
            else if (gamepad1.dpad_right){
                end_pos = 1;
                middle = "Middle";
                left = "Left";
                right = "R\u0332i\u0332g\u0332h\u0332t\u0332";
            }
            if (gamepad1.touchpad){
                break;
            }
        }



        telemetry.addData("Robot has initialized", ")");
        telemetry.addData("end pos", end_pos);
        telemetry.addData("time delay", wait_time);
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

                //blue side
                if (valLeft == max || valMid == max || valRight == max) {
                    delay_wait(wait_time);
                    VisionTest.pos state = VisionTest.pos.left;
                    if (valMid == max){
                        state = VisionTest.pos.middle;
                    }
                    else if (valRight == max){
                        state = VisionTest.pos.right;
                    }
                    sleep(10);


                    if (state == VisionTest.pos.left) {
                        Actions.runBlocking(BlueL_To_Tape);
                        dropPixel();
                        Actions.runBlocking(Blue_to_waitL);
                    }
                    else if (state == VisionTest.pos.right) {
                        Actions.runBlocking(BlueR_To_Tape);
                        dropPixel();
                        Actions.runBlocking(Blue_to_wait);
                    }
                    else if (state == VisionTest.pos.middle) {
                        Actions.runBlocking(BlueM_To_Tape);
                        dropPixel();
                        Actions.runBlocking(Blue_to_wait);
                    }


                    Actions.runBlocking(Blue_Return);
                    arm_slide.setPower(1);
                    sleep(400);
                    arm_slide.setPower(0);


                    if (state == VisionTest.pos.left) {
                        Actions.runBlocking(Place_On_board_BlueL);
                    }
                    else if (state == VisionTest.pos.right) {
                        Actions.runBlocking(Place_On_board_BlueR);
                    }
                    else if (state == VisionTest.pos.middle) {
                        Actions.runBlocking(Place_On_board_BlueM);
                    }

                    sleep(10);
                    top_grip.setPosition(0);
                    wait_stop.reset();
                    sleep(10);

                    while (true){
                        wiggle();
                        if (wait_stop.seconds() > 1){
                            break;
                        }
                    }

                    sleep(50);
                    if (end_pos == 0) {
                        Actions.runBlocking(Blue_Place_returnL);
                    }
                    else if (end_pos == 1) {
                        Actions.runBlocking(Blue_Place_returnR);
                    }
                    sleep(12345678);
                }




                //red side=================================================================

                if (valLeftR == max || valMidR == max || valRightR == max) {
                    VisionTest.pos state = VisionTest.pos.left;
                    delay_wait(wait_time);
                    if (valMidR == max){
                        state = VisionTest.pos.middle;
                    }
                    else if (valRightR == max){
                        state = VisionTest.pos.right;
                    }

                    sleep(10);

                    if (state == VisionTest.pos.left) {
                        Actions.runBlocking(RedL_To_Tape);
                        dropPixel();
                        Actions.runBlocking(Red_to_wait);
                    }
                    else if (state == VisionTest.pos.right) {
                        Actions.runBlocking(RedR_To_Tape);
                        dropPixel();
                        Actions.runBlocking(Red_to_waitR);
                    }
                    else if (state == VisionTest.pos.middle) {
                        Actions.runBlocking(RedM_To_Tape);
                        dropPixel();
                        Actions.runBlocking(Red_to_wait);
                    }


                    Actions.runBlocking(Red_Return);
                    arm_slide.setPower(1);
                    sleep(400);
                    arm_slide.setPower(0);


                    if (state == VisionTest.pos.left) {
                        Actions.runBlocking(Place_On_board_RedL);
                    }
                    else if (state == VisionTest.pos.right) {
                        Actions.runBlocking(Place_On_board_RedR);
                    }
                    else if (state == VisionTest.pos.middle) {
                        Actions.runBlocking(Place_On_board_RedM);
                    }

                    sleep(10);
                    top_grip.setPosition(0);
                    wait_stop.reset();
                    sleep(10);

                    while (true){
                        wiggle();
                        if (wait_stop.seconds() > 1){
                            break;
                        }
                    }

                    sleep(50);
                    if (end_pos == 0) {
                        Actions.runBlocking(Red_Place_returnL);
                    }
                    else if (end_pos == 1) {
                        Actions.runBlocking(Red_Place_returnR);
                    }
                    sleep(1233456);
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
        hand_tilt.setPosition(0.2);

    }

    public void delay_wait(int wait_dela) {
        wait_stop.reset();
        while(true){
            if (wait_stop.seconds() > wait_dela){
                break;
            }
        }
    }


    public void wiggle() {
        int frequency = 15;
        arm_slide.setPower(-1);
        bottomleftmotor.setPower(-1);
        bottomrightmotor.setPower(-1);
        topleftmotor.setPower(-1);
        toprightmotor.setPower(-1);
        sleep(frequency);
        arm_slide.setPower(1);
        bottomleftmotor.setPower(1);
        bottomrightmotor.setPower(1);
        topleftmotor.setPower(1);
        toprightmotor.setPower(1);
        sleep(frequency);
        arm_slide.setPower(0);
        bottomleftmotor.setPower(0);
        bottomrightmotor.setPower(0);
        topleftmotor.setPower(0);
        toprightmotor.setPower(0);

    }
}