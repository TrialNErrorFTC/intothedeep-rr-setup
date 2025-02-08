package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTesting {


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity drive = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 16)
                .setDimensions(16.25,16)
                .setStartPose(new Pose2d(-22+13.5,72-10, Math.toRadians(180)))
                .build();


        VelConstraint sonicVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(150),
                new AngularVelConstraint(Math.PI)
        ));
        AccelConstraint sonicAcc = new ProfileAccelConstraint(-100, 100);
//
        //        VelConstraint fastVel = new MinVelConstraint(Arrays.asList(
//                new TranslationalVelConstraint(80),
//                new AngularVelConstraint(Math.PI)
//        ));
//        AccelConstraint fastAcc = new ProfileAccelConstraint(-45, 70);
//
        VelConstraint baseVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(110),
                new AngularVelConstraint(Math.PI)
        ));
        AccelConstraint baseAcc = new ProfileAccelConstraint(-120, 120);

        VelConstraint travelVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(90),
                new AngularVelConstraint(Math.PI)
        ));
        AccelConstraint travelAcc = new ProfileAccelConstraint(-60, 60);


//        VelConstraint sonicVel = null;
//        AccelConstraint sonicAcc = null;
        VelConstraint fastVel = null;
        AccelConstraint fastAcc = null;
//        VelConstraint baseVel = null;
//        AccelConstraint baseAcc = null;

        double pp = 53;


        TrajectoryActionBuilder clip1 = drive.getDrive().actionBuilder(drive.getPose())
                .splineToLinearHeading(new Pose2d(-2, 31, Math.toRadians(90)), Math.toRadians(270), null, null)
                .endTrajectory();

        TrajectoryActionBuilder pushBlocks1 = clip1.fresh()
                .setReversed(false)
                //.splineToLinearHeading(new Pose2d(-23.5,37, Math.toRadians(90)), Math.toRadians(170), travelVel, travelAcc)
                //.splineToLinearHeading(new Pose2d(-45,7, Math.toRadians(90)), Math.toRadians(-180), travelVel, travelAcc)

                .splineTo(new Vector2d(-23.5,37), Math.toRadians(170), travelVel, travelAcc)
                .splineTo(new Vector2d(-45,7), Math.toRadians(-180), travelVel, travelAcc)
                .turnTo(Math.toRadians(90))

                .setReversed(true)
                .strafeTo(new Vector2d(-45,45), sonicVel, sonicAcc)

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-55,7), Math.toRadians(-150), fastVel, fastAcc)
                .setReversed(false)
                .strafeTo(new Vector2d(-55,49), sonicVel, sonicAcc)
                //.waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder moveToPickup2 = pushBlocks1.fresh()
                .strafeTo(new Vector2d(-55, 38), baseVel, baseAcc)
                // Move to wall pickup
                .strafeTo(new Vector2d(-55, pp), baseVel, baseAcc)
                //.waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder moveToClip2 = moveToPickup2.fresh()
                .setReversed(true)
                .splineTo(new Vector2d(-4, 30), Math.toRadians(270), fastVel, fastAcc)
                //.waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder moveToPickup3 = moveToClip2.fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-48, pp-10), Math.toRadians(90), baseVel, baseAcc)
                .strafeTo(new Vector2d(-48, pp), travelVel, baseAcc)
                //.waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder moveToClip3 = moveToPickup3.fresh()
                //.waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(-2, 30), Math.toRadians(270), baseVel, baseAcc)
                .endTrajectory();

        TrajectoryActionBuilder moveToPickup4 = moveToClip3.fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-48, pp-10), Math.toRadians(90), baseVel, baseAcc)
                .strafeTo(new Vector2d(-48, pp), travelVel, baseAcc)
                .endTrajectory();

        TrajectoryActionBuilder moveToClip4 = moveToPickup4.fresh()
                //.waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(0, 30), Math.toRadians(270), baseVel, baseAcc)
                .endTrajectory();

        TrajectoryActionBuilder moveToPickup5 = moveToClip4.fresh();
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(-48, pp), Math.toRadians(90), baseVel, baseAcc)
//                .endTrajectory();

        TrajectoryActionBuilder moveToClip5 = moveToPickup5.fresh();
        //.waitSeconds(1)
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(2, 30), Math.toRadians(270), baseVel, baseAcc)
//                .endTrajectory();


        // Build all actions before waitForStart()
        Action clip1Action = clip1.build();
        Action pushBlocks1Action = pushBlocks1.build();
        Action moveToPickup2Action = moveToPickup2.build();
        Action moveToClip2Action = moveToClip2.build();
        Action moveToPickup3Action = moveToPickup3.build();
        Action moveToClip3Action = moveToClip3.build();
        Action moveToPickup4Action = moveToPickup4.build();
        Action moveToClip4Action = moveToClip4.build();
        Action moveToPickup5Action = moveToPickup5.build();
        Action moveToClip5Action = moveToClip5.build();

        drive.runAction(
                new SequentialAction(
                        clip1Action,
                        pushBlocks1Action,
                        moveToPickup2Action,
                        moveToClip2Action,
                        moveToPickup3Action,
                        moveToClip3Action,
                        moveToPickup4Action,
                        moveToClip4Action,
                        moveToPickup5Action,
                        moveToClip5Action
                )
        );

//        myBot.runAction(myBot.getDrive().actionBuilder(myBot.getPose())
//                .splineToLinearHeading(new Pose2d(-2, 30, Math.toRadians(90)), Math.toRadians(270))
//
//                .waitSeconds(1.5)
//
//                //.strafeTo(new Vector2d(-2, 32))
//                .splineToLinearHeading(new Pose2d(-27,36, Math.toRadians(90)), Math.toRadians(195))
//                .splineToLinearHeading(new Pose2d(-45,10, Math.toRadians(90)), Math.toRadians(180))
//
//                .setReversed(true)
//                .strafeTo(new Vector2d(-45,49))
//
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-55,11), Math.toRadians(-145))
//                .strafeTo(new Vector2d(-55,49))
//
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-64,10), Math.toRadians(-145))
//                .strafeTo(new Vector2d(-64.5,49))
//
//                .strafeTo(new Vector2d(-64.5, 43))
//
//                // Move to wall pickup
//                .strafeTo(new Vector2d(-64.5, pp))
//                // Pickup
//
//                // --Clip shit--
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-4, 30), Math.toRadians(270))
//                // Clip 1
//
//                                .waitSeconds(1.25)
//                // Move to pickup
//                //.setReversed(true)
//                .splineToConstantHeading(new Vector2d(-48, pp), Math.toRadians(90))
//                //.strafeTo(new Vector2d(-48, pp))
//                // Pickup 2
//
//                // Move to Clip 2
//                //.strafeTo(new Vector2d(-35, 45))
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-2, 30), Math.toRadians(270))
//                //Clip 2
//
//                .waitSeconds(1.5)
//
//                // Move to pickup 3
//                //.strafeTo(new Vector2d(-2, 45))
//                .splineToConstantHeading(new Vector2d(-48, pp), Math.toRadians(90))
//                //.strafeTo(new Vector2d(-48, pp))
//                //pickup 3
//
//                // Move to Clip 3
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(0, 30), Math.toRadians(270))
//                //Clip 3
//
//                .waitSeconds(1.5)
//
//                // Move to pickup 4
//                .splineToConstantHeading(new Vector2d(-48, pp), Math.toRadians(90))
//                //.strafeTo(new Vector2d(-48, pp))
//                //pickup
//
//                // Move to clip 4
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(2, 30), Math.toRadians(270))
//                //Clip 4
//
////                .strafeTo(new Vector2d(3, 38))
////                .strafeTo(new Vector2d(3, 30))
////                .turnTo(Math.toRadians(270))
//                .build());

        // Run shit for first clip


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(drive)
                .start();
    }
}