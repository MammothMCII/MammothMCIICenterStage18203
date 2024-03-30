package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11)
                .setDimensions(13, 14)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-38.5, -63, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-46, -34), Math.toRadians(90), null, new ProfileAccelConstraint(-10, 10))
                .waitSeconds(1)
                .lineToYConstantHeading(-50)
                .splineToConstantHeading(new Vector2d(-58, -50), Math.toRadians(90), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10, 10))
                .splineToConstantHeading(new Vector2d(-58, -24), Math.toRadians(90), null, new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(-50, -9, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-10, 10))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(30, -9), 0)
                .splineToConstantHeading(new Vector2d(40, -20), 0)
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(52, -29), 0, null, new ProfileAccelConstraint(-10, 10))
                .waitSeconds(1)
                .lineToXConstantHeading(45)
                //.splineToConstantHeading(new Vector2d(35, -10), 0)
                .splineToConstantHeading(new Vector2d(35, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(53, -10), 0)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}