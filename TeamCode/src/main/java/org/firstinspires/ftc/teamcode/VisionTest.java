package org.firstinspires.ftc.teamcode;


import static androidx.core.math.MathUtils.clamp;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.common.subtyping.qual.Bottom;
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


@Autonomous(name= "AutoMain", group="autonomous")
//@Disabled//comment out this line before using
public class VisionTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime wait_stop = new ElapsedTime();


    //-1 for debug, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static int valMidR = -1;
    private static int valLeftR = -1;
    private static int valRightR = -1;

    private static int max = 0;

    private static int wait_time = 0;

    private static int side = 0; //0 = front, 1 = back

    private static int color = 0; //0 = red, 1 = blue

    private static int end_pos = 0; //0 = left,   1 = right,   2 = middle


    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;

    private static final float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static final float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static final float[] midPos = {4f / 8f + offsetX, 4.1f / 8f + offsetY};//0 = col, 1 = row
    private static final float[] leftPos = {0.5f / 8f + offsetX, 4f / 8f + offsetY};
    private static final float[] rightPos = {7.5f / 8f + offsetX, 4f / 8f + offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 1280;
    private final int cols = 720;

    OpenCvCamera phoneCam;


    private DcMotor arm_tilt;
    private DcMotor topleftmotor;
    private DcMotor bottomleftmotor;
    private DcMotor toprightmotor;
    private DcMotor bottomrightmotor;


    enum pos {
        left,
        middle,
        right
    }

    //arm lift
    public class Lift {
        private DcMotor lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "arm_slide");
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 600.0) { //50 ticks per in
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }
    }


    //top claw
    public class TopGrip{
        private Servo topgrip;

        public TopGrip(HardwareMap hardwareMap){
            topgrip = hardwareMap.get(Servo.class, "top_grip");
        }

        public class Close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                topgrip.setPosition(0);
                return false;
            }
        }

        public Action Close() { return new Close(); }

        public class Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                topgrip.setPosition(0.4);
                return false;
            }
        }
        public Action Open() { return new Open(); }

    }

    //bottom grip
    public class BottomGrip {
        private Servo bottomgrip;

        public BottomGrip(HardwareMap hardwareMap){
            bottomgrip = hardwareMap.get(Servo.class, "bottom_grip");
        }

        public class Close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                bottomgrip.setPosition(0.4);
                return false;
            }
        }

        public Action Close() { return new Close(); }

        public class Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                bottomgrip.setPosition(0);
                return false;
            }
        }
        public Action Open() { return new Open(); }

    }

    public class Wrist {
        private Servo wrist;

        public Wrist(HardwareMap hardwareMap){
            wrist = hardwareMap.get(Servo.class, "hand_tilt");
        }

        public class Up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                wrist.setPosition(0.2);
                return false;
            }
        }

        public Action Up() { return new Up(); }

        public class Down implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                wrist.setPosition(0.48);
                return false;
            }
        }
        public Action Down() { return new Down(); }

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
        arm_tilt = hardwareMap.get(DcMotor.class, "arm_tilt");

        //initAprilTag();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //visionPortal.close();
        phoneCam.openCameraDevice();//open camera

        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        arm_tilt = hardwareMap.get(DcMotor.class, "arm_tilt");


        //telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));   #ref
        // this is the park and wait selection
        selectOptions();
        telemetry.addLine("calculating trajectories");
        telemetry.update();

        Lift lift = new Lift(hardwareMap);
        TopGrip topGrip = new TopGrip(hardwareMap);
        BottomGrip bottomGrip = new BottomGrip(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        //roadrunner initialization
        MecanumDrive drive = null;

        Action PixelAction = null;
        Action ToWaitAction = null;
        Action ToBackdropAction = null;
        Action BackdropPlaceAction = null;
        Action ParkAction = null;

        Pose2d startPoseRed = new Pose2d(-38.5, -63, Math.toRadians(90));

        Pose2d startPoseBlue = new Pose2d(-32, 63, Math.toRadians(-90));

        Pose2d startPoseRedBack = new Pose2d(9, -63, Math.toRadians(90));

        Pose2d startPoseBlueBack = new Pose2d(17, 63, Math.toRadians(-90));

        if (side == 0 && color == 0){ //red front
            drive = new MecanumDrive(hardwareMap, startPoseRed);
        }
        else if (side == 0 && color == 1){ //blue front
            drive = new MecanumDrive(hardwareMap, startPoseBlue);
        }

        else if (side == 1 && color == 0){ //red back
            drive = new MecanumDrive(hardwareMap, startPoseRedBack);
        }
        else if (side == 1 && color == 1){ //blue back
            drive = new MecanumDrive(hardwareMap, startPoseBlueBack);
        }




        //build trajectories --------------------------------------------------------

        // red to tape
        Action RedL_To_Tape = drive.actionBuilder(startPoseRed)
                .splineToConstantHeading(new Vector2d(-46, -34), Math.toRadians(90), null, new ProfileAccelConstraint(-10, 10))
                .build();

        Action RedM_To_Tape = drive.actionBuilder(startPoseRed)
                .splineToConstantHeading(new Vector2d(-38, -32), Math.toRadians(90), null, new ProfileAccelConstraint(-10, 10))
                .build();

        Action RedR_To_Tape = drive.actionBuilder(startPoseRed)
                .splineToConstantHeading(new Vector2d(-23, -36), Math.toRadians(0), null, new ProfileAccelConstraint(-10, 10))
                .build();


        // red to wait pos
        Action Red_to_waitL = drive.actionBuilder(new Pose2d(-46, -34, Math.toRadians(90)))
                .lineToYConstantHeading(-50)
                .splineToConstantHeading(new Vector2d(-58, -50), Math.toRadians(90), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10, 10))
                .splineToConstantHeading(new Vector2d(-58, -24), Math.toRadians(90), null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(-50, -9, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-10, 10))
                .build();


        Action Red_to_waitM = drive.actionBuilder(new Pose2d(-38, -32, Math.toRadians(90)))
                .lineToYConstantHeading(-40)
                .splineToConstantHeading(new Vector2d(-58, -40), Math.toRadians(90), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10, 10))
                .splineToConstantHeading(new Vector2d(-58, -24), Math.toRadians(90), null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(-50, -9, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-10, 10))
                .build();

        Action Red_to_waitR = drive.actionBuilder(new Pose2d(-23, -36, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-26, -33), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, -33), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-60, -24), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-50, -9, Math.toRadians(0)), Math.toRadians(0))
                .build();


        //red return
        Action Red_Return = drive.actionBuilder(new Pose2d(-50, -9, 0))
                .splineToConstantHeading(new Vector2d(30, -9), 0)
                .splineToConstantHeading(new Vector2d(40, -20), 0)
                .build();

        //red board
        Action Place_On_board_RedL = drive.actionBuilder(new Pose2d(40, -20, 0))
                .splineToConstantHeading(new Vector2d(53, -29), 0, null, new ProfileAccelConstraint(-10, 10))
                .build();

        Action Place_On_board_RedM = drive.actionBuilder(new Pose2d(40, -20, 0))
                .splineToConstantHeading(new Vector2d(53, -35), 0, null, new ProfileAccelConstraint(-10, 10))
                .build();

        Action Place_On_board_RedR = drive.actionBuilder(new Pose2d(40, -20, 0))
                .splineToConstantHeading(new Vector2d(53, -42), 0, null, new ProfileAccelConstraint(-10, 10))
                .build();


        //park
        Action Red_Place_returnL = drive.actionBuilder(new Pose2d(53, -31, 0))
                .lineToXConstantHeading(45)
                .splineToConstantHeading(new Vector2d(35, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(53, -10), 0)
                .build();

        Action Red_Place_returnR = drive.actionBuilder(new Pose2d(53, -31, 0))
                .lineToXConstantHeading(45)
                .splineToConstantHeading(new Vector2d(35, -40), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(53, -60), 0)
                .build();


        ///--------------------------------------------------------------------------------------------------
        // Blue R to tape
        Action BlueR_To_Tape = drive.actionBuilder(startPoseBlue)
                .splineToConstantHeading(new Vector2d(-46, 38), Math.toRadians(-90), null, new ProfileAccelConstraint(-10, 10))
                .build();

        Action BlueL_To_Tape = drive.actionBuilder(startPoseBlue)
                .splineToConstantHeading(new Vector2d(-24, 36), Math.toRadians(0), null, new ProfileAccelConstraint(-10, 10))
                .build();

        Action BlueM_To_Tape = drive.actionBuilder(startPoseBlue)
                .splineToConstantHeading(new Vector2d(-38, 32), Math.toRadians(-90), null, new ProfileAccelConstraint(-10, 10))
                .build();


        // Blue to wait pos
        Action Blue_to_waitL = drive.actionBuilder(new Pose2d(-24, 36, Math.toRadians(-90)))
                .splineToConstantHeading(new Vector2d(-26, 33), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, 33), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-60, 24), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-50, 9, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Action Blue_to_waitM = drive.actionBuilder(new Pose2d(-38, 32, Math.toRadians(-90)))
                .lineToYConstantHeading(40)
                .splineToConstantHeading(new Vector2d(-58, 40), Math.toRadians(-90), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10, 10))
                .splineToConstantHeading(new Vector2d(-58, 24), Math.toRadians(-90), null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(-50, 9, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-10, 10))
                .build();

        Action Blue_to_waitR = drive.actionBuilder(new Pose2d(-46, 38, Math.toRadians(-90)))
                .lineToYConstantHeading(50)
                .splineToConstantHeading(new Vector2d(-58, 50), Math.toRadians(-90), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10, 10))
                .splineToConstantHeading(new Vector2d(-58, 24), Math.toRadians(-90), null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(-50, 9, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-10, 10))
                .build();

        //Blue return
        Action Blue_Return = drive.actionBuilder(new Pose2d(-50, 9, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(30, 9), 0)
                .splineToConstantHeading(new Vector2d(40, 20), 0)
                .build();


        //Blue board
        Action Place_On_board_BlueR = drive.actionBuilder(new Pose2d(40, 20, 0))
                .splineToConstantHeading(new Vector2d(54, 29), 0, null, new ProfileAccelConstraint(-10, 10))
                .build();

        Action Place_On_board_BlueM = drive.actionBuilder(new Pose2d(40, 20, 0))
                .splineToConstantHeading(new Vector2d(54, 35), 0, null, new ProfileAccelConstraint(-10, 10))
                .build();

        Action Place_On_board_BlueL = drive.actionBuilder(new Pose2d(40, 20, 0))
                .splineToConstantHeading(new Vector2d(54, 42), 0, null, new ProfileAccelConstraint(-10, 10))
                .build();



        //park
        Action Blue_Place_returnL = drive.actionBuilder(new Pose2d(53, 31, 0))
                .lineToXConstantHeading(45)
                .splineToConstantHeading(new Vector2d(35, 40), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(53, 60), 0)
                .build();

        Action Blue_Place_returnR = drive.actionBuilder(new Pose2d(53, 31, 0))
                .lineToXConstantHeading(45)
                .splineToConstantHeading(new Vector2d(35, 15), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(53, 10), 0)
                .build();

        ///----------------------------------------------------------------------------------------------------





        telemetry.addData("Robot has initialized", ")");
        telemetry.addData("end pos", end_pos);
        telemetry.addData("time delay", wait_time);
        telemetry.update();

        Actions.runBlocking(new SequentialAction(
                bottomGrip.Close(), topGrip.Close()));

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values B", valLeft + "   " + valMid + "   " + valRight);
            telemetry.addData("Values R", valLeftR + "   " + valMidR + "   " + valRightR);

            telemetry.update();

            while (valLeft != -1 || valRight != -1 || valMid != -1 || valLeftR != -1 || valRightR != -1 || valMidR != -1) {
                telemetry.addData("Values B", valLeft + "   " + valMid + "   " + valRight);
                telemetry.addData("Values R", valLeftR + "   " + valMidR + "   " + valRightR);
                telemetry.addData("max", max);
                telemetry.update();


                if (isStopRequested()) return;


                 //blue side
                 if (color == 1) {

                     pos state = pos.left;
                     if (valMid == max) {
                         state = pos.middle;
                     } else if (valRight == max) {
                         state = pos.right;
                     }

                     //front
                     if (state == pos.left){
                         PixelAction = BlueL_To_Tape;
                         BackdropPlaceAction = Place_On_board_BlueL;
                         ToWaitAction = Blue_to_waitL;
                     }
                     else if (state == pos.middle){
                         PixelAction = BlueM_To_Tape;
                         BackdropPlaceAction = Place_On_board_BlueM;
                         ToWaitAction = Blue_to_waitM;
                     }
                     else if (state == pos.right){
                         PixelAction = BlueR_To_Tape;
                         BackdropPlaceAction = Place_On_board_BlueR;
                         ToWaitAction = Blue_to_waitR;
                     }

                     ToBackdropAction = Blue_Return;


                     if (end_pos == 0){
                         ParkAction = Blue_Place_returnL;
                     }
                     else if (end_pos == 1) {
                         ParkAction = Blue_Place_returnR;
                     }

                 }



                 //red side
                 if (color == 0) {

                     pos state = pos.left;
                     if (valMidR == max) {
                        state = pos.middle;
                     } else if (valRightR == max) {
                        state = pos.right;
                     }


                     //front
                     if (state == pos.left){
                         PixelAction = RedL_To_Tape;
                         BackdropPlaceAction = Place_On_board_RedL;
                         ToWaitAction = Red_to_waitL;
                     }
                     else if (state == pos.middle){
                         PixelAction = RedM_To_Tape;
                         BackdropPlaceAction = Place_On_board_RedM;
                         ToWaitAction = Red_to_waitM;
                     }
                     else if (state == pos.right){
                         PixelAction = RedR_To_Tape;
                         BackdropPlaceAction = Place_On_board_RedR;
                         ToWaitAction = Red_to_waitR;
                     }

                     ToBackdropAction = Red_Return;


                     if (end_pos == 0){
                         ParkAction = Red_Place_returnL;
                     }
                     else if (end_pos == 1) {
                         ParkAction = Red_Place_returnR;
                     }

                 }

                //run set cations
                Actions.runBlocking(
                        new SequentialAction(

                                PixelAction,
                                bottomGrip.Open(),
                                new ParallelAction(
                                        ToWaitAction,
                                        wrist.Up()
                                ),
                                ToBackdropAction,
                                //lift.liftUp(),
                                new ParallelAction(
                                        lift.liftUp(),
                                        BackdropPlaceAction
                                ),
                                topGrip.Open(),
                                ParkAction
                        )
                );
                 sleep(29247977);


            }

        }
        //visionPortal.close();
        phoneCam.closeCameraDevice();
    }


    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat yCbCrR = new Mat();
        Mat thresholdMat = new Mat();
        Mat thresholdR = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
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


            double[] pixMid = yCbCrChan2Mat.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
            valMid = (int) pixMid[0];

            double[] pixLeft = yCbCrChan2Mat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
            valLeft = (int) pixLeft[0];

            double[] pixRight = yCbCrChan2Mat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
            valRight = (int) pixRight[0];

            double[] pixMidR = yCbCrR.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
            valMidR = (int) pixMidR[0];

            double[] pixLeftR = yCbCrR.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
            valLeftR = (int) pixLeftR[0];

            double[] pixRightR = yCbCrR.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
            valRightR = (int) pixRightR[0];


            max = Math.max(valMid, Math.max(valLeft, Math.max(valRight, Math.max(valLeftR, Math.max(valMidR, valRightR)))));


            //create three points
            Point pointMid = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
            Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
            Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointLeft, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointRight, 5, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols() * (leftPos[0] - rectWidth / 2),
                            input.rows() * (leftPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (leftPos[0] + rectWidth / 2),
                            input.rows() * (leftPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols() * (midPos[0] - rectWidth / 2),
                            input.rows() * (midPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (midPos[0] + rectWidth / 2),
                            input.rows() * (midPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols() * (rightPos[0] - rectWidth / 2),
                            input.rows() * (rightPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (rightPos[0] + rectWidth / 2),
                            input.rows() * (rightPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }

                default: {
                    return input;
                }
            }
        }

    }

    public void wiggle() {
        int frequency = 15;

        bottomleftmotor.setPower(-1);
        bottomrightmotor.setPower(-1);
        topleftmotor.setPower(-1);
        toprightmotor.setPower(-1);
        sleep(frequency);

        bottomleftmotor.setPower(1);
        bottomrightmotor.setPower(1);
        topleftmotor.setPower(1);
        toprightmotor.setPower(1);
        sleep(frequency);

        bottomleftmotor.setPower(0);
        bottomrightmotor.setPower(0);
        topleftmotor.setPower(0);
        toprightmotor.setPower(0);

    }

    private void selectOptions() {
        while (!isStarted() && !isStopRequested()) {
            String middle = "Middle";
            String left = "Left";
            String right = "Right";
            String three = "3";
            String no = "no";
            String five = "5";
            String tmax = "max";

            int progress = 0;
            int screen_count = 2;
            boolean held = false;

            while (progress <= screen_count) {
                //telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));   #ref
                // this is the park and wait selection
                if (progress == 0) {
                    telemetry.addLine(String.format("░░░░░■%s░┌---------┐░░░▲:%s░░░░░", middle, three));
                    telemetry.addLine(String.format("░%s■=╩=■%s| B    F  |░■:%s░░O:%s░", left, right, no, five));
                    telemetry.addLine(String.format("░░░░░░░░░░░└---------┘░░░░X:%s░░", tmax));
                    telemetry.addLine("DO NOT HIT START");

                    telemetry.update();

                    if (gamepad1.x) {
                        wait_time = 0;
                        three = "3";
                        no = "n̲o̲";
                        five = "5";
                        tmax = "max";
                    } else if (gamepad1.y) {
                        wait_time = 3;
                        three = "3\u0332";
                        no = "no";
                        five = "5";
                        tmax = "max";
                    } else if (gamepad1.a) {
                        wait_time = 7;
                        three = "3";
                        no = "no";
                        five = "5";
                        tmax = "m̲a̲x̲";
                    } else if (gamepad1.b) {
                        wait_time = 5;
                        three = "3";
                        no = "no";
                        five = "5\u0332";
                        tmax = "max";
                    }
                    if (gamepad1.dpad_left) {
                        end_pos = 0;
                        middle = "Middle";
                        left = "L̲e̲f̲t̲";
                        right = "Right";
                    } else if (gamepad1.dpad_up) {
                        end_pos = 2;
                        middle = "M̲i̲d̲d̲l̲e̲";
                        left = "Left";
                        right = "Right";
                    } else if (gamepad1.dpad_right) {
                        end_pos = 1;
                        middle = "Middle";
                        left = "Left";
                        right = "R̲i̲g̲h̲t̲";
                    }
                }
                if (progress == 1) {
                    String BlueF = "  ";
                    String RedF = "  ";
                    String BlueB = "  ";
                    String RedB = "  ";
                    if (color == 0 && side == 0){
                        RedF = "■";
                    }
                    else if (color == 1 && side == 0){
                        BlueF = "■";
                    }
                    else if (color == 0 && side == 1){
                        RedB = "■";
                    }
                    else if (color == 1 && side == 1){
                        BlueB = "■";
                    }
                    telemetry.addLine("selecting bit");
                    telemetry.addLine("-------");
                    telemetry.addLine(String.format("|   %s|%s  ||", BlueF, BlueB));
                    telemetry.addLine("|   |   |");
                    telemetry.addLine(String.format("|   %s|%s  ||", RedF, RedB));
                    telemetry.addLine("-------");
                    telemetry.addData("side", side);
                    telemetry.addData("color", color);

                    telemetry.update();

                    if (gamepad1.dpad_left) {
                        color = 0;

                    } else if (gamepad1.dpad_up) {
                        side = 0;

                    } else if (gamepad1.dpad_right) {
                        color = 1;

                    } else if (gamepad1.dpad_down) {
                        side = 1;

                    }


                }
                if (gamepad1.touchpad && !held) {

                    held = true;

                    if (gamepad1.touchpad_finger_1_x > 0.5) {
                        progress++;
                    } else {
                        progress -= 1;
                        progress = clamp(progress, 0, screen_count);
                    }
                }
                if (!gamepad1.touchpad) {
                    held = false;
                }

            }
            break;
        }
    }
}
