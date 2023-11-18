package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "AutoRed", group = "Autonomous")
public class SlightlyLessBasicAutoLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-12, -60, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory Traj1 = drive.trajectoryBuilder(startPose)
                .strafeLeft(25)
                .build();

        Trajectory Traj2 = drive.trajectoryBuilder(Traj1.end())
                .forward(68)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(Traj1);
        drive.followTrajectory(Traj2);
    }
}