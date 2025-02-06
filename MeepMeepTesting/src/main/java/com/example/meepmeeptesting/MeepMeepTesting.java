package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double pp = 53;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 80, Math.toRadians(180), Math.toRadians(180), 16)
                .setDimensions(16.25,16)
                .setStartPose(new Pose2d(-22+13.5,72-10, Math.toRadians(180)))
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(myBot.getPose())
                .splineToLinearHeading(new Pose2d(-2, 30, Math.toRadians(90)), Math.toRadians(270))

                .waitSeconds(1.5)

                //.strafeTo(new Vector2d(-2, 32))
                .splineToLinearHeading(new Pose2d(-27,36, Math.toRadians(90)), Math.toRadians(195))
                .splineToLinearHeading(new Pose2d(-45,10, Math.toRadians(90)), Math.toRadians(180))

                .setReversed(true)
                .strafeTo(new Vector2d(-45,49))

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-55,11), Math.toRadians(-145))
                .strafeTo(new Vector2d(-55,49))

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-64,10), Math.toRadians(-145))
                .strafeTo(new Vector2d(-64.5,49))

                .strafeTo(new Vector2d(-64.5, 43))

                // Move to wall pickup
                .strafeTo(new Vector2d(-64.5, pp))
                // Pickup

                // --Clip shit--
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-4, 30), Math.toRadians(270))
                // Clip 1

                                .waitSeconds(1.25)
                // Move to pickup
                //.setReversed(true)
                .splineToConstantHeading(new Vector2d(-48, pp), Math.toRadians(90))
                //.strafeTo(new Vector2d(-48, pp))
                // Pickup 2

                // Move to Clip 2
                //.strafeTo(new Vector2d(-35, 45))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-2, 30), Math.toRadians(270))
                //Clip 2

                .waitSeconds(1.5)

                // Move to pickup 3
                //.strafeTo(new Vector2d(-2, 45))
                .splineToConstantHeading(new Vector2d(-48, pp), Math.toRadians(90))
                //.strafeTo(new Vector2d(-48, pp))
                //pickup 3

                // Move to Clip 3
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0, 30), Math.toRadians(270))
                //Clip 3

                .waitSeconds(1.5)

                // Move to pickup 4
                .splineToConstantHeading(new Vector2d(-48, pp), Math.toRadians(90))
                //.strafeTo(new Vector2d(-48, pp))
                //pickup

                // Move to clip 4
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(2, 30), Math.toRadians(270))
                //Clip 4

//                .strafeTo(new Vector2d(3, 38))
//                .strafeTo(new Vector2d(3, 30))
//                .turnTo(Math.toRadians(270))
                .build());

        // Run shit for first clip


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}