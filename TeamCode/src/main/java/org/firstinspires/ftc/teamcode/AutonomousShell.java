package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "Autonomous_Sad", group = "Autonomous")
@Disabled
public class AutonomousShell extends LinearOpMode {

    String Signal_State = "no";


    private DcMotor bottomleftmotor = null;
    private DcMotor topleftmotor = null;
    private DcMotor toprightmotor = null;
    private DcMotor bottomrightmotor = null;
    private BNO055IMU imu         = null;      // Control/Expansion Hub IMU
    private DcMotor Arm;
    private TouchSensor armsafetybutton;
    private TouchSensor armlimitbutton;
    private ColorSensor cool;
    private DcMotor arm_Tilt;
    private Servo scoop;
    private Servo stab;
    private Servo planeLauncher;

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;


    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 480 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     ARM_COUNTS_PER_MOTOR_REV    = 480 ;    // eg: TETRIX Motor Encoder
    static final double     ARM_DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     ARM_WHEEL_DIAMETER_INCHES   = 7 ;     // For figuring circumference
    static final double     ARM_COUNTS_PER_INCH     = (ARM_COUNTS_PER_MOTOR_REV * ARM_DRIVE_GEAR_REDUCTION) /
            (ARM_WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.5;
    static final double     HEADING_THRESHOLD       = 0 ;  // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 9;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private double  targetHeading = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;

    //double armMaxGround = Arm.getCurrentPosition() - ARM_COUNTS_PER_MOTOR_REV * 2;


    @Override
    public void runOpMode() {
        bottomleftmotor = hardwareMap.get(DcMotor.class, "bottom left motor");
        topleftmotor = hardwareMap.get(DcMotor.class, "top left motor");
        toprightmotor = hardwareMap.get(DcMotor.class, "top right motor");
        bottomrightmotor = hardwareMap.get(DcMotor.class, "bottom right motor");
        scoop = hardwareMap.get(Servo.class, "scoop");
        stab = hardwareMap.get(Servo.class, "stab");
        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        arm_Tilt = hardwareMap.get(DcMotor.class, "armTilt");
        armsafetybutton = hardwareMap.get(TouchSensor.class, "arm safety button");
        armlimitbutton = hardwareMap.get(TouchSensor.class, "arm limit button");
        cool = hardwareMap.get(ColorSensor.class, "cool");

        bottomleftmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        topleftmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        toprightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        bottomleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        toprightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bottomleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        toprightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_Tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        bottomleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        toprightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();
            //this line here is really the only important bit
            while (opModeIsActive()) {
                encoderDrive(32, 10, 0.25, 0.5, 1, 0.25);
                encoderDrive(-2, 10, 0.25, 0.5, 1, 0.25);
                sleep(8080008);
            }
        }
    }



    public void encoderDrive(
            double DriveDistance,
            double timeoutS, double MinSpeed, double MaxSpeed, double RampRate, double RampBounds) {
        double speed = MinSpeed;
        int newTopLeftTarget;
        int newTopRightTarget;
        int newBotLeftTarget;
        int newBotRightTarget;
        double AvgPos;
        double AvgTarget;
        double TicksToVelocity = 0;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTopLeftTarget = topleftmotor.getCurrentPosition() + (int)(DriveDistance * COUNTS_PER_INCH);
            newTopRightTarget = toprightmotor.getCurrentPosition() + (int)(DriveDistance * COUNTS_PER_INCH);
            newBotLeftTarget = bottomleftmotor.getCurrentPosition() + (int)(DriveDistance * COUNTS_PER_INCH);
            newBotRightTarget = bottomrightmotor.getCurrentPosition() + (int)(DriveDistance * COUNTS_PER_INCH);

            bottomleftmotor.setTargetPosition(newBotLeftTarget);
            topleftmotor.setTargetPosition(newTopLeftTarget);
            toprightmotor.setTargetPosition(newTopRightTarget);
            bottomrightmotor.setTargetPosition(newBotRightTarget);


            // Turn On RUN_TO_POSITION
            bottomleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            toprightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            bottomleftmotor.setPower(Math.abs(speed));
            topleftmotor.setPower(Math.abs(speed));
            toprightmotor.setPower(Math.abs(speed));
            bottomrightmotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (bottomleftmotor.isBusy() || topleftmotor.isBusy() || toprightmotor.isBusy() || bottomrightmotor.isBusy())) {
                AvgPos = (topleftmotor.getCurrentPosition() + toprightmotor.getCurrentPosition() + bottomleftmotor.getCurrentPosition() + bottomrightmotor.getCurrentPosition()) / 4;
                AvgTarget = (newTopLeftTarget + newTopRightTarget + newBotLeftTarget + newBotRightTarget) / 4;
                if (AvgPos <= RampBounds * AvgTarget && AvgPos <= 1/2 * AvgTarget) {
                    speed += RampRate;
                    if (speed >= MaxSpeed){
                        speed = MaxSpeed;
                        if (TicksToVelocity == 0) {
                            TicksToVelocity = AvgPos; //this is horrible and almost completely unredeable. I don't know how this worked so well
                        }// I am reading over this 6 months later and I have no clue how this works, this code is actually magic, what does ramp bounds do?
                    } else if (AvgPos > AvgTarget - TicksToVelocity){
                        speed += RampRate;
                        if (speed <= MinSpeed){
                            speed = MinSpeed;
                        }
                    } //todo: start writing better code

                    bottomleftmotor.setPower(Math.abs(speed));
                    topleftmotor.setPower(Math.abs(speed));
                    toprightmotor.setPower(Math.abs(speed));
                    bottomrightmotor.setPower(Math.abs(speed));
                    telemetry.update();
                }

            }

            // Stop all motion;
            bottomleftmotor.setPower(0);
            topleftmotor.setPower(0);
            toprightmotor.setPower(0);
            bottomrightmotor.setPower(0);

            // Turn off RUN_TO_POSITION
            bottomleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            toprightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    public void CloseHand(){
        scoop.setPosition(0.7); //close hand
        stab.setPosition(0.3); //close hand
    }
    public void OpenHand(){
        scoop.setPosition(0.1);
        stab.setPosition(0.9);
    }

    public void ArmMoveSeconds(double armSpeed, long seconds, long delay) {
        arm_Tilt.setPower(armSpeed);
        Arm.setPower(-(armSpeed));
        sleep(seconds);
        arm_Tilt.setPower(0);
        Arm.setPower(0);
        sleep(delay);
    }

    public void RunToZero() {
        while (!armsafetybutton.isPressed()) {
            arm_Tilt.setPower(-1);
            Arm.setPower(1);
            if (armlimitbutton.isPressed()){
                break;
            }
        }
    }
    public void RunToTop() {
        while (!armlimitbutton.isPressed()) {
            arm_Tilt.setPower(1);
            Arm.setPower(-1);
            if (armlimitbutton.isPressed()) {  //this checks for the wrong variable to break but it still works, now its worse
                break;
            }
        }
        //below is all the code for gyro drive



    }

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading, double timeoutS) {

        int newTopLeftTarget;
        int newTopRightTarget;
        int newBotLeftTarget;
        int newBotRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            newTopLeftTarget = topleftmotor.getCurrentPosition() + moveCounts;
            newTopRightTarget = toprightmotor.getCurrentPosition() + moveCounts;
            newBotLeftTarget = bottomleftmotor.getCurrentPosition() + moveCounts;
            newBotRightTarget = bottomrightmotor.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            bottomleftmotor.setTargetPosition(newBotLeftTarget);
            topleftmotor.setTargetPosition(newTopLeftTarget);
            toprightmotor.setTargetPosition(newTopRightTarget);
            bottomrightmotor.setTargetPosition(newBotRightTarget);

            bottomleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            toprightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (bottomleftmotor.isBusy() || topleftmotor.isBusy() || toprightmotor.isBusy() || bottomrightmotor.isBusy())) {
                telemetry.update();

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            bottomleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            toprightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void turnToHeading(double maxTurnSpeed, double heading, double timeoutS) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        runtime.reset();


        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD) && runtime.seconds() < timeoutS) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);


            // Clip the speed to the maximum permitted value.
            turnSpeed = (Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed) * ((Math.abs(headingOffset) / heading) + 0.2));

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }


    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > TURN_SPEED/2)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        bottomleftmotor.setPower(leftSpeed);
        topleftmotor.setPower(leftSpeed);
        toprightmotor.setPower(rightSpeed);
        bottomrightmotor.setPower(rightSpeed);
    }

    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      topleftmotor.getCurrentPosition(),
                    toprightmotor.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
    public void encoderLift(double speed,
                            double TargetInches,
                            double timeoutS) {
        int newArmTarget;
        int newarm_TiltTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newArmTarget = topleftmotor.getCurrentPosition() - (int)(TargetInches * ARM_COUNTS_PER_INCH);
            newarm_TiltTarget = topleftmotor.getCurrentPosition() + (int)(TargetInches * ARM_COUNTS_PER_INCH);

            Arm.setTargetPosition(newArmTarget);
            arm_Tilt.setTargetPosition(newarm_TiltTarget);


            // Turn On RUN_TO_POSITION
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_Tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            Arm.setPower((Math.abs(speed)));
            arm_Tilt.setPower(-(Math.abs(speed)));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Arm.isBusy() && arm_Tilt.isBusy())) {
                telemetry.update();
            }

            // Stop all motion;
            Arm.setPower(0);
            arm_Tilt.setPower(0);

            // Turn off RUN_TO_POSITION
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm_Tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            sleep(250);   // optional pause after each move.
        }
    }
    public void iHopeThisWorks(double maxTurnSpeed, double heading, double timeoutS){
        if (!(cool.red() >= 150)) {
            // Run getSteeringCorrection() once to pre-calculate the current error
            getSteeringCorrection(heading, P_DRIVE_GAIN);

            runtime.reset();


            // keep looping while we are still active, and not on heading.


            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);


            // Clip the speed to the maximum permitted value.
            turnSpeed = (Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed) * ((Math.abs(headingOffset) / heading) + 0.2));

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);


            // Stop all motion;
            moveRobot(0, 0);
        }

    }
}






