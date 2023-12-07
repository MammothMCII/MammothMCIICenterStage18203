
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.AutonomousConstants;


@Autonomous(name = "AutoBlue2", group = "Autonomous")
public class SlightlyLessBasicAutoRight2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-12, 60, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory Traj1 = drive.trajectoryBuilder(startPose)
                .strafeRight(27)
                .build();

        Trajectory Traj2 = drive.trajectoryBuilder(Traj1.end())
                .forward(AutonomousConstants.Forward_distance)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(Traj1);
        drive.followTrajectory(Traj2);
    }
}