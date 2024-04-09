package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public final class turningTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();
            while (true){
                Actions.runBlocking(
                        drive.actionBuilder(beginPose)

                                .turn(Math.toRadians(90))
                                .lineToYConstantHeading(36)
                                .turn(Math.toRadians(90))
                                .lineToXConstantHeading(-36)
                                .turn(Math.toRadians(90))
                                .lineToYConstantHeading(0)
                                .turn(Math.toRadians(90))
                                .lineToXConstantHeading(0)
                                 /**
                                .turn(2*Math.PI)
                                .waitSeconds(123456)
                                **/
                                .build());
                sleep(233254445);
            }
        }
    }
}
