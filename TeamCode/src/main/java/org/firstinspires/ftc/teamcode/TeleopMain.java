package org.firstinspires.ftc.teamcode;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;





@TeleOp(name = "TeleopMain")
public class TeleopMain extends LinearOpMode {
    ElapsedTime safetytimer = new ElapsedTime();
    ElapsedTime wristup = new ElapsedTime();
    boolean reseting = false;
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

    private double safteytime = 800;

    private int pixels_held = 0;

    private double pixelheightadjusted = 25;

    mode state = mode.place;
    mode laststate = null;
    input lastinput = null;
    public enum mode {
        neutral,
        pickup,
        liftup,
        place,
        liftdown,
        safteypixel,
        pixel,
    }

    public enum input {
        rB,
        y,
        b,
    }

    //bottom grip vars
    double bottom_grip_closed = 0.38;
    double bottom_grip_pickup = 0.2;
    double bottom_grip_drop = 0.2;

    //top grip vars
    double top_grip_closed = 0.11;
    double top_grip_pickup = 0.5;
    double top_grip_drop = .25;

    //wrist vars
    double wrist_down = 0.48;
    double wrist_neutral = 0.1;
    double wrist_place = 0.22;

    boolean rbpressed = false;
    boolean bpressed = false;
    boolean ypressed = false;

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
    boolean dropToggle = false;


    boolean mode1toggle = false;
    boolean mode2toggle = false;

    double encoderGroundPos = 0.0;
    double lasttickpos = 0.0;

    double encoderstartpos = 0.0;






    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {


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


        bottom_grip.setPosition(bottom_grip_closed);
        top_grip.setPosition(top_grip_closed);
        winch_release.setPosition(1);

        hand_tilt.setPosition(0.48);
        arm_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        planeLauncher.setPosition(0);





        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

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

                arm_power = gamepad1.left_stick_y;

                if (arm_power < 0.4 && arm_power > -0.4) {
                    arm_power = 0;
                }
                else if (!(state == mode.place) && !gamepad1.left_bumper) {
                    arm_power = 0;
                }


                if (armsafetybutton.isPressed()) {
                    arm_slide.setPower(Math.min(Math.max(arm_power, -1), 0));
                    encoderGroundPos = arm_slide.getCurrentPosition();
                }
                else if(armlimitbutton.isPressed()) {
                    arm_slide.setPower((Math.min(Math.max(arm_power, 0), -1)));
                }
                else {
                    arm_slide.setPower(arm_power);
                }

                if (gamepad1.dpad_left && !winchToggle) {
                    if (!winchOn) {winch_release.setPosition(0.4); winchOn = true;}
                    else {winch_release.setPosition(1);

                        winchOn = false;
                    }
                    winchToggle = true;
                }
                else if (!gamepad1.dpad_left) winchToggle = false;

                //wiggle
                if (gamepad1.x) wiggle();

                if (gamepad1.dpad_right && !planeToggle) {
                    if (!planeOn) {
                        planeLauncher.setPosition(0.5);
                        planeOn = true;
                    }
                    else {planeLauncher.setPosition(0);

                        planeOn = false;
                    }
                    planeToggle = true;
                }
                else if (!gamepad1.dpad_right) planeToggle = false;



                switch_state_constants();

                //change state conditions
                switch (state) {

                    case liftdown:
                        if ((safetytimer.milliseconds() >= safteytime) || armsafetybutton.isPressed()) {
                            switchstate();
                            encoderGroundPos = arm_slide.getCurrentPosition();
                        }

                    case safteypixel:
                        if (!armsafetybutton.isPressed()) {
                            switchstate();
                        }

                    case pixel:
                        if ((arm_slide.getCurrentPosition() - lasttickpos) <= -pixelheightadjusted) {
                            switchstate();
                        }

                    case liftup:
                        if (arm_slide.getCurrentPosition() < (-1000.0 - encoderstartpos)){
                            switchstate();
                        }

                }

                if (gamepad1.right_bumper && !rbpressed) {
                    lastinput = input.rB;
                    switchstate();
                    rbpressed =true;
                }
                if (!gamepad1.right_bumper) rbpressed = false;

                if (gamepad1.b && !bpressed) {
                    lastinput = input.b;
                    switchstate();
                    bpressed =true;
                }
                if (!gamepad1.b) bpressed = false;

                if (gamepad1.y && !ypressed) {
                    lastinput = input.y;
                    switchstate();
                    ypressed =true;
                }
                if (!gamepad1.y) ypressed = false;



                telemetry.addData("current state", state);
                telemetry.addData("last", laststate);
                telemetry.addData("encoder pos", arm_slide.getCurrentPosition());
                telemetry.addData("pixels held", pixels_held);
                telemetry.addData("wristup", wristup.milliseconds());





                telemetry.update();
            }

        }

    }


    public void switchstate() {
        //store last state
        mode lastlaststate = laststate;

        //state switching logic
        switch (state){

            case neutral:
                if (gamepad1.y) {
                    encoderstartpos = arm_slide.getCurrentPosition();
                    laststate = state;
                    state = mode.liftup;
                    return;

                }
                if (gamepad1.right_bumper) {
                    laststate = state;
                    state = mode.pickup;
                    return;
                }
                break;


            case pickup:
                if (gamepad1.b && armsafetybutton.isPressed()) {
                    laststate = state;
                    state = mode.safteypixel;
                    encoderGroundPos = arm_slide.getCurrentPosition();
                    return;
                }

                if (gamepad1.b && !armsafetybutton.isPressed()) {
                    laststate = state;
                    state = mode.pixel;
                    encoderstartpos = arm_slide.getCurrentPosition();
                    return;
                }
                if (gamepad1.right_bumper) {
                    wristup.reset();
                    pixels_held = 2;
                    laststate = state;
                    state = mode.neutral;
                    return;
                }
                break;

            case safteypixel:
                if (!armsafetybutton.isPressed()) {
                    laststate = state;
                    state = mode.pixel;
                    encoderstartpos = arm_slide.getCurrentPosition();
                    return;
                }
                break;

            case pixel:
                if ((arm_slide.getCurrentPosition() - encoderstartpos) <= -pixelheightadjusted) {
                    laststate = state;
                    state = mode.pickup;
                    return;
                }
                break;


            case liftup:
                laststate = state;
                state = mode.place;
                break;

            case place:
                if (gamepad1.right_bumper) {
                    if (pixels_held == 2) {
                        bottom_grip.setPosition(bottom_grip_drop);
                        pixels_held = 1;
                        return;
                    }
                    if (pixels_held == 1) {
                        top_grip.setPosition(top_grip_drop);
                        pixels_held = 0;
                        return;
                    }
                    if (pixels_held == 0) {
                        //hand to adjust position
                        bottom_grip.setPosition(bottom_grip_closed);
                        top_grip.setPosition(.6);
                        hand_tilt.setPosition(wrist_down);
                        return;
                    }
                    return;
                }


                if (gamepad1.y) {
                    laststate = state;
                    safetytimer.reset();
                    state = mode.liftdown;
                    return;
                }
                break;

            case liftdown:
                if (armsafetybutton.isPressed() || safetytimer.milliseconds() >= safteytime) {
                    laststate = state;
                    state = mode.neutral;
                    encoderGroundPos = arm_slide.getCurrentPosition();
                    return;}
                break;
        }
        //restore laststate if state is not changed
        laststate = lastlaststate;
    }


    public void switch_state_constants(){
        //state constants
        switch (state) {

            case neutral:

                top_grip.setPosition(top_grip_closed);
                bottom_grip.setPosition(bottom_grip_closed);
                if (wristup.milliseconds() >=1000) {
                    hand_tilt.setPosition(wrist_neutral);
                }
                else {
                    hand_tilt.setPosition(wrist_down);
                }
                return;

            case pickup:
                hand_tilt.setPosition(wrist_down);
                top_grip.setPosition(top_grip_pickup);
                bottom_grip.setPosition(bottom_grip_pickup);
                return;

            case place:
                if (pixels_held != 0) {
                    hand_tilt.setPosition(wrist_place);
                }
                return;

            case liftup:
                arm_slide.setPower(-1);
                top_grip.setPosition(top_grip_closed);
                bottom_grip.setPosition(bottom_grip_closed);
                hand_tilt.setPosition(wrist_place);
                return;

            case liftdown:
                arm_slide.setPower(1);
                top_grip.setPosition(top_grip_closed);
                bottom_grip.setPosition(bottom_grip_closed);
                hand_tilt.setPosition(wrist_neutral);
                return;

            case safteypixel:
                arm_slide.setPower(-.4);
                return;

            case pixel:
                arm_slide.setPower(-.4);
                if (laststate == mode.safteypixel) {
                    pixelheightadjusted = 0;
                }
                else {
                    pixelheightadjusted = 28;
                }
        }

    }


/**
        private void LiftUp(double inches, double speed) {
        boolean initilized = false;
        double inchesadjusted = inches;

        if (armsafetybutton.isPressed()) {inchesadjusted = (inches - 0.1);}
        while (armsafetybutton.isPressed()) {arm_slide.setPower(-.3);}
        if (!armsafetybutton.isPressed()) {initilized = true; arm_slide.setPower(0);}


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
 **/


    public void wiggle() {
        int frequency = 1;
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