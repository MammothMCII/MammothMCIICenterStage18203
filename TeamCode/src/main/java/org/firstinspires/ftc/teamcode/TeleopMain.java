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
    private DcMotor Arm;
    private DcMotor arm_Tilt;
    private DcMotor winch;
    private Servo scoop;
    private Servo stab;
    private Servo planeLauncher;



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
        boolean planeToggle = false;
        boolean planeHeld = false;

        topleftmotor = hardwareMap.get(DcMotor.class, "top left motor");
        bottomleftmotor = hardwareMap.get(DcMotor.class, "bottom left motor");
        toprightmotor = hardwareMap.get(DcMotor.class, "top right motor");
        bottomrightmotor = hardwareMap.get(DcMotor.class, "bottom right motor");
        armsafetybutton = hardwareMap.get(TouchSensor.class, "arm safety button");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        arm_Tilt = hardwareMap.get(DcMotor.class, "armTilt");
        scoop = hardwareMap.get(Servo.class, "scoop");
        stab = hardwareMap.get(Servo.class, "stab");
        armlimitbutton = hardwareMap.get(TouchSensor.class, "arm limit button");
        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");
        winch = hardwareMap.get(DcMotor.class, "winch");

        // set motor directions on initialization
        topleftmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomleftmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        toprightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_Tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                    arm_power = 0;
                }

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

                arm_Tilt.setPower(arm_tilt_speed);

                // opens grabber
                if (gamepad1.left_bumper) {
                    scoop.setPosition(0.7);
                    stab.setPosition(0.3);
                }
                // closes grabber
                if (gamepad1.right_bumper) {
                    scoop.setPosition(0.1);
                    stab.setPosition(0.9);
                }
                if (gamepad1.a && planeToggle == true && planeHeld == false){
                    planeLauncher.setPosition(0);
                    planeToggle = true;
                    planeHeld = true;
                }
                if (gamepad1.a && planeToggle == true && planeHeld == false){
                    planeLauncher.setPosition(1);
                    planeToggle = false;
                    planeHeld = true;
                }
                if (!gamepad1.a){
                    planeHeld = false;
                }



                telemetry.update();
            }
        }
    }
}