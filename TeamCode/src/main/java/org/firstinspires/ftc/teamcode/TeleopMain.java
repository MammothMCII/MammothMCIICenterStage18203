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
    private DcMotor Arm1;
    private Servo lefthand;
    private Servo righthand;



    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double drive_speed;
        double turbo;
        double arm_speed;
        double arm_power = 0;

        topleftmotor = hardwareMap.get(DcMotor.class, "top left motor");
        bottomleftmotor = hardwareMap.get(DcMotor.class, "bottom left motor");
        toprightmotor = hardwareMap.get(DcMotor.class, "top right motor");
        bottomrightmotor = hardwareMap.get(DcMotor.class, "bottom right motor");
        armsafetybutton = hardwareMap.get(TouchSensor.class, "arm safety button");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");
        lefthand = hardwareMap.get(Servo.class, "left hand");
        righthand = hardwareMap.get(Servo.class, "right hand");
        armlimitbutton = hardwareMap.get(TouchSensor.class, "arm limit button");

        // set motor directions on initialization
        topleftmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomleftmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        toprightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                drive_speed = 0.5;
                drive_speed = (1 - gamepad1.left_trigger);
                drive_speed = (gamepad1.right_trigger);

                // this handles macanum wheel driving with strafe
                // multiplied by drive speed
                bottomleftmotor.setPower(drive_speed * (((0 - gamepad1.right_stick_y) + gamepad1.left_stick_x) - gamepad1.right_stick_x));
                bottomrightmotor.setPower(drive_speed * (((0 - gamepad1.right_stick_y) - gamepad1.left_stick_x) + gamepad1.right_stick_x));
                topleftmotor.setPower(drive_speed * ((0 - gamepad1.right_stick_y) + gamepad1.left_stick_x + gamepad1.right_stick_x));
                toprightmotor.setPower(drive_speed * (((0 - gamepad1.right_stick_y) - gamepad1.left_stick_x) - gamepad1.right_stick_x));

                if (gamepad1.dpad_up) {
                    arm_power = -1;
                }
                else if (gamepad1.dpad_down) {
                    arm_power = 1;
                }
                else{
                    arm_power = 0;
                }
                //this will be the real stuff
                if (armsafetybutton.isPressed()) {
                    Arm1.setPower(-(Math.min(Math.max(arm_power, -1), 0)));
                    Arm.setPower(Math.min(Math.max(arm_power, -1), 0));
                }
                else if(armlimitbutton.isPressed()) {
                    Arm1.setPower(-(Math.min(Math.max(arm_power, 0), 1)));
                    Arm.setPower((Math.min(Math.max(arm_power, 0), 1)));
                }
                else {
                    Arm1.setPower(-(arm_power));
                    Arm.setPower(2 * arm_power);
                }

                // opens grabber
                if (gamepad1.left_bumper) {
                    lefthand.setPosition(0.7);
                    righthand.setPosition(0.3);
                }
                // closes grabber
                if (gamepad1.right_bumper) {
                    lefthand.setPosition(0.1);
                    righthand.setPosition(0.9);
                }

                telemetry.update();
            }
        }
    }
}