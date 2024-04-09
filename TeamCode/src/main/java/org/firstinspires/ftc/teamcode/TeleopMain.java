package org.firstinspires.ftc.teamcode;

import static androidx.core.math.MathUtils.clamp;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;



@TeleOp(name = "TeleopMain")
public class TeleopMain extends LinearOpMode {

    private DcMotor topleftmotor;
    private DcMotor bottomleftmotor;
    private DcMotor toprightmotor;
    private DcMotor bottomrightmotor;
    private TouchSensor armsafetybutton;
    private TouchSensor armlimitbutton;
    private DcMotor winch;

    private Servo planeLauncher;
    private Servo winch_release;
    private Servo top_grip;
    private Servo hand_tilt;
    private Servo bottom_grip;
    private DcMotor arm_slide;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;




    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double drive_speed;
        double turbo;
        double arm_speed;
        double arm_power = 0;
        double arm_tilt_speed = 0;
        boolean launcher = false;
        double launcher_timer = 0;
        boolean winchToggle = false;
        boolean winchOn = false;
        boolean planeToggle = false;
        boolean planeOn = false;
        boolean topToggle = false;
        boolean topOn = false;
        boolean bottomToggle = false;
        boolean bottomOn = false;
        boolean tiltToggle = false;
        boolean tiltOn = false;
        boolean onepixeltoggle = false;
        int pixels_held = 0;
        boolean dropToggle = false;

        int mode = 2;
        boolean mode1toggle = false;
        boolean mode2toggle = false;

        ElapsedTime mode0timer = new ElapsedTime();
        boolean tiltup = false;

        //initAprilTag();

        topleftmotor = hardwareMap.get(DcMotor.class, "top left motor");
        bottomleftmotor = hardwareMap.get(DcMotor.class, "bottom left motor");
        toprightmotor = hardwareMap.get(DcMotor.class, "top right motor");
        bottomrightmotor = hardwareMap.get(DcMotor.class, "bottom right motor");
        armsafetybutton = hardwareMap.get(TouchSensor.class, "arm safety button");

        armlimitbutton = hardwareMap.get(TouchSensor.class, "arm limit button");
        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");
        winch_release = hardwareMap.get(Servo.class, "winch release");
        top_grip = hardwareMap.get(Servo.class, "top_grip");
        bottom_grip = hardwareMap.get(Servo.class, "bottom_grip");
        hand_tilt = hardwareMap.get(Servo.class, "hand_tilt");
        winch = hardwareMap.get(DcMotor.class, "winch");
        arm_slide = hardwareMap.get(DcMotor.class, "arm_slide");

        // set motor directions on initialization
        topleftmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomleftmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        toprightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);


        bottom_grip.setPosition(0.38);
        top_grip.setPosition(0.11);
        winch_release.setPosition(1);

        hand_tilt.setPosition(0.46);
        arm_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //telemetryAprilTag();
                telemetry.update();

                drive_speed = (1 - (gamepad1.right_trigger/1.5));


                // this handles macanum wheel driving with strafe
                // multiplied by drive speed
                bottomleftmotor.setPower(drive_speed * (((0 - gamepad1.right_stick_y) + gamepad1.left_stick_x) - gamepad1.right_stick_x));
                bottomrightmotor.setPower(drive_speed * (((0 - gamepad1.right_stick_y) - gamepad1.left_stick_x) + gamepad1.right_stick_x));
                topleftmotor.setPower(drive_speed * ((0 - gamepad1.right_stick_y) + gamepad1.left_stick_x + gamepad1.right_stick_x));
                toprightmotor.setPower(drive_speed * (((0 - gamepad1.right_stick_y) - gamepad1.left_stick_x) - gamepad1.right_stick_x));

                if (gamepad1.dpad_up) {
                    winch.setPower(1);
                }
                else if (gamepad1.dpad_down) {
                    winch.setPower(-1);
                }
                else{
                    winch.setPower(0);
                }

                if (mode == 2 || gamepad1.left_bumper) {
                    arm_power = gamepad1.left_stick_y;
                }
                if (arm_power > 0.4) {
                    clamp(arm_power, 0.4, 1);
                }
                else if (arm_power < -0.4) {
                    clamp(arm_power, -1, -0.4);
                }
                else {
                    arm_power = 0;
                }

                if (armsafetybutton.isPressed()) {
                    arm_slide.setPower(Math.min(Math.max(arm_power, -1), 0));
                }
                else if(armlimitbutton.isPressed()) {
                    arm_slide.setPower((Math.min(Math.max(arm_power, 0), -1)));
                }
                else {
                    arm_slide.setPower(arm_power);
                }


                if (gamepad1.b && !onepixeltoggle && mode == 1) {
                    LiftUp(.5,.25);
                    onepixeltoggle = true;
                }
                else if (!gamepad1.b) onepixeltoggle = false;

                if (gamepad1.dpad_left && !winchToggle) {
                    if (!winchOn) {winch_release.setPosition(0.4); winchOn = true;}
                    else {winch_release.setPosition(1);

                        winchOn = false;
                    }
                    winchToggle = true;
                }
                else if (!gamepad1.dpad_left) winchToggle = false;



                //mode 1 pickup toggle
                if (gamepad1.right_bumper && !mode1toggle) {
                    if (mode == 0) {
                        top_grip.setPosition(.45);
                        bottom_grip.setPosition(0.05);
                        hand_tilt.setPosition(.46);
                        pixels_held = 0;
                        mode = 1;
                    }
                    else if (mode == 1) {
                        top_grip.setPosition(.11);
                        bottom_grip.setPosition(0.38);
                        mode0timer.reset();
                        tiltup = true;
                        pixels_held = 2;
                        mode = 0;
                    }
                    mode1toggle = true;
                }
                else if (!gamepad1.right_bumper) mode1toggle = false;

                //delay for hand tilt after pickup
                if (mode0timer.milliseconds() >= 300 && tiltup) {
                    hand_tilt.setPosition(0);
                    tiltup = false;
                }

                if (gamepad1.right_bumper && !dropToggle && mode == 2) {
                    if (pixels_held == 1) {
                        top_grip.setPosition(.3);
                        pixels_held = 0;
                    }
                    else if (pixels_held == 2) {
                        bottom_grip.setPosition(0.2);
                        pixels_held = 1;
                    }
                    dropToggle = true;
                }
                else if (!gamepad1.right_bumper) dropToggle = false;

                if (gamepad1.y && !mode2toggle) {
                    if (mode == 0) {
                        hand_tilt.setPosition(0.2);
                        LiftUp(10, 1);
                        mode = 2;
                    }
                    else if (mode == 2) {
                        hand_tilt.setPosition(0);
                        bottom_grip.setPosition(0.38);
                        top_grip.setPosition(0.11);
                        resetarm();
                        mode = 0;
                    }
                    mode2toggle = true;
                }
                else if (!gamepad1.y) mode2toggle = false;

                //wiggle
                if (gamepad1.x) {
                    wiggle();

                }

                if (gamepad1.touchpad) {
                    bottom_grip.setPosition(0);
                    top_grip.setPosition(1);
                }

                if (gamepad1.dpad_right && !planeToggle) {
                    if (!planeOn) {planeLauncher.setPosition(0.5); planeOn = true;}
                    else {planeLauncher.setPosition(0);

                        planeOn = false;
                    }
                    planeToggle = true;
                }
                else if (!gamepad1.dpad_right) planeToggle = false;


                if (gamepad1.a){
                    hand_tilt.setPosition(0);
                }



                telemetry.update();
            }

        }
        //visionPortal.close();
    }


    public void LiftUp(double inches, double speed) {
        boolean initilized = false;
        double inchesadjusted = inches;
        if (!initilized) {
            if (armsafetybutton.isPressed()) {inchesadjusted = (inches - 0.1);}
            while (armsafetybutton.isPressed()) {arm_slide.setPower(-.3);}
            if (!armsafetybutton.isPressed()) {initilized = true; arm_slide.setPower(0);}
        }

        if (initilized) {
            double inchesticks = (50 * inchesadjusted);
            double initpos = arm_slide.getCurrentPosition();
            while ((arm_slide.getCurrentPosition()-initpos) >= -inchesticks) {
                arm_slide.setPower(-speed);
                if ((arm_slide.getCurrentPosition()-initpos) <= -inchesticks) { //50 ticks per in
                    arm_slide.setPower(0);
                }
            }
        }
    }



    public void resetarm() {
        if (!armsafetybutton.isPressed()) {
            ElapsedTime safetytimer = new ElapsedTime();
            safetytimer.reset();
            while (!armsafetybutton.isPressed() && safetytimer.milliseconds() <= 600) {
                arm_slide.setPower(1);
                if (armsafetybutton.isPressed() || safetytimer.milliseconds() >= 600) {
                    arm_slide.setPower(0);
                    }
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

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(1430 , 1430 , 480, 620)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "FrontCamera"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        //telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        //telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        //telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    private void lockToBackdrop() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double average_offset = 0;
        double average_distance = 0;
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                //telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                //telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                average_offset += detection.ftcPose.yaw;
                average_distance += detection.ftcPose.range;
            }
        }   // end for() loop
        average_offset = average_offset/currentDetections.size();
        average_distance = average_distance/currentDetections.size();

        double motor_power = 0;
        double forward_power = 0;

        motor_power = average_offset/20;

        if (average_distance > 10){
            forward_power = average_distance/20;
        }

        bottomleftmotor.setPower(-motor_power + forward_power);
        topleftmotor.setPower(-motor_power + forward_power);
        bottomrightmotor.setPower(motor_power + forward_power);
        toprightmotor.setPower(motor_power + forward_power);




    }   // end method telemetryAprilTag()



}