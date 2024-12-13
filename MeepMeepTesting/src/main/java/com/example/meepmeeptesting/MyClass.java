package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(11, -15, Math.toRadians(43)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(28, -4, Math.toRadians(43)), Math.toRadians(45))
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(0, -25, Math.toRadians(0)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(0, -39, Math.toRadians(0)), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(7.5, -39), Math.toRadians(0))
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(28, -4), Math.toRadians(43))
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(5, -25, Math.toRadians(0)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(5, -40, Math.toRadians(0)), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(18, -40), Math.toRadians(0))
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(28, -4), Math.toRadians(43))
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(15, -25, Math.toRadians(0)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(15, -40, Math.toRadians(0)), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(22.5, -40), Math.toRadians(0))
                .waitSeconds(.5)
//                .strafeToLinearHeading(new Vector2d(28, -4), Math.toRadians(43))
                .setTangent(135)
                .splineToSplineHeading(new Pose2d(11, -30, Math.toRadians(43)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(28, -4, Math.toRadians(43)), Math.toRadians(45))
                .waitSeconds(1)
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}