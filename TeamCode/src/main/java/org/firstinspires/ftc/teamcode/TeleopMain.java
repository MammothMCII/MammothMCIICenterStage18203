package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.checkerframework.checker.units.qual.A;

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

        bottom_grip.setPosition(0);
        top_grip.setPosition(1);


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                drive_speed = (0.5 + (gamepad1.right_trigger/2) - (gamepad1.left_trigger/3));

                if (gamepad1.touchpad_finger_1_x > 0) {
                    // touchpad arm controlls, very cool
                    //todo: make this work for two fingers if this is actually what we do
                    arm_power = -gamepad1.touchpad_finger_1_y/4;
                }

                if (gamepad1.touchpad_finger_1_x < 0) {
                    arm_tilt_speed = -gamepad1.touchpad_finger_1_y/3;
                }

                if (!gamepad1.touchpad_finger_1){
                    arm_power = 0;
                    arm_tilt_speed = 0;
                }

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

                if (gamepad1.b && !armlimitbutton.isPressed()) {
                    arm_slide.setPower(1);
                }
                else if (gamepad1.a && !armsafetybutton.isPressed()) {
                    arm_slide.setPower(-1);
                }
                else{
                    arm_slide.setPower(0);
                }

                if (gamepad1.dpad_left && !winchToggle) {
                    if (winchOn == false) {winch_release.setPosition(1); winchOn = true;}
                    else {winch_release.setPosition(0.4);

                        winchOn = false;
                    }
                    winchToggle = true;
                }
                else if (!gamepad1.dpad_left) winchToggle = false;



                if (gamepad1.left_bumper && !topToggle) {
                    if (topOn == false) {top_grip.setPosition(0); topOn = true;}
                    else {top_grip.setPosition(1);

                        topOn = false;
                    }
                    topToggle = true;
                }
                else if (!gamepad1.left_bumper) topToggle = false;


                if (gamepad1.right_bumper && !bottomToggle) {
                    if (bottomOn == false) {bottom_grip.setPosition(1); bottomOn = true;}
                    else {bottom_grip.setPosition(0);

                        bottomOn = false;
                    }
                    bottomToggle = true;
                }
                else if (!gamepad1.right_bumper) bottomToggle = false;



                if (gamepad1.y && !tiltToggle) {
                    if (tiltOn == false) {hand_tilt.setPosition(0.2); tiltOn = true;}
                    else {hand_tilt.setPosition(0.48);

                        tiltOn = false;
                    }
                    tiltToggle = true;
                }
                else if (!gamepad1.y) tiltToggle = false;


                //wiggle
                if (gamepad1.x) {
                    wiggle();
                }


                /**
                //this will be the real stuff
                if (armsafetybutton.isPressed()) {

                    Arm.setPower(Math.min(Math.max(arm_power, -1), 0));
                }
                else if(armlimitbutton.isPressed()) {

                    Arm.setPower((Math.min(Math.max(arm_power, 0), 1)));
                }
                else {

                    Arm.setPower(arm_power);
                }
                **/

                if (gamepad1.dpad_right && !planeToggle) {
                    if (planeOn == false) {planeLauncher.setPosition(1); planeOn = true;}
                    else {planeLauncher.setPosition(0);

                        planeOn = false;
                    }
                    planeToggle = true;
                }
                else if (!gamepad1.dpad_right) planeToggle = false;





                telemetry.update();
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