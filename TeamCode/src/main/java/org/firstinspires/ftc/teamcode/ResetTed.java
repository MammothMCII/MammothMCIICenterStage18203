package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ResetTed")

public class ResetTed extends LinearOpMode {


    private TouchSensor armsafetybutton;


    private Servo hand_tilt;

    private DcMotor arm_slide;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        armsafetybutton = hardwareMap.get(TouchSensor.class, "arm safety button");

        hand_tilt = hardwareMap.get(Servo.class, "hand_tilt");

        arm_slide = hardwareMap.get(DcMotor.class, "arm_slide");
        arm_slide.setDirection(DcMotorSimple.Direction.REVERSE);




        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            hand_tilt.setPosition(0.5);

            sleep(1000);
            resetarm();
        }
    }


    private void resetarm() {
        ElapsedTime safetytimer = new ElapsedTime();
        safetytimer.reset();
        while (!armsafetybutton.isPressed() && safetytimer.milliseconds() < 1000) {
            arm_slide.setPower(-1);
            if (armsafetybutton.isPressed() | safetytimer.milliseconds() >= 1000) {
                arm_slide.setPower(0);
                break;
            }
        }
    }
}
