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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-24, 37, Math.toRadians(-90)))
                .lineToYConstantHeading(39)
                .splineToConstantHeading(new Vector2d(-26, 39), Math.toRadians(180), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10, 10))
                .splineToConstantHeading(new Vector2d(-57, 33), Math.toRadians(-90), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-10, 10))
                .splineToConstantHeading(new Vector2d(-57, 24), Math.toRadians(-90), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-10, 10))
                .splineToSplineHeading(new Pose2d(-50, 9, Math.toRadians(0)), Math.toRadians(0))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}