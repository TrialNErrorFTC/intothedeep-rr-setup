package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.reflect.Array;
import java.util.Arrays;

@Autonomous
public class ClipAuto extends TeleOpActionsRR {
    FtcDashboard dash = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
//    private int pp = 53;
//
//    private VelConstraint sonicVel = new MinVelConstraint(Arrays.asList(
//            new TranslationalVelConstraint(70.0),
//            new AngularVelConstraint(Math.PI)
//    ));
//    private AccelConstraint sonicAcc = new ProfileAccelConstraint(-30, 60);
//
//    private VelConstraint fastVel = new MinVelConstraint(Arrays.asList(
//            new TranslationalVelConstraint(50.0),
//            new AngularVelConstraint(Math.PI / 1.2)
//    ));
//    private AccelConstraint fastAcc = new ProfileAccelConstraint(-15, 50);
//
//    private VelConstraint baseVel = new MinVelConstraint(Arrays.asList(
//            new TranslationalVelConstraint(40.0),
//            new AngularVelConstraint(Math.PI / 1.5)
//    ));
//    private AccelConstraint baseAcc = new ProfileAccelConstraint(-15, 40);

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-22+13.5,72-10, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ActionControl actionControl = new ActionControl();
        Pose2d pose;

        VelConstraint sonicVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(120),
                new AngularVelConstraint(Math.PI)
        ));
        AccelConstraint sonicAcc = new ProfileAccelConstraint(-45, 70);

        VelConstraint fastVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(100),
                new AngularVelConstraint(Math.PI)
        ));
        AccelConstraint fastAcc = new ProfileAccelConstraint(-45, 60);

        VelConstraint baseVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(70),
                new AngularVelConstraint(Math.PI)
        ));
        AccelConstraint baseAcc = new ProfileAccelConstraint(-30, 20);

        double pp = 53;

        TrajectoryActionBuilder clip1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(-2, 30, Math.toRadians(90)), Math.toRadians(270), baseVel, baseAcc)
                .endTrajectory();

        TrajectoryActionBuilder pushBlocks1 = clip1.fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-27,36, Math.toRadians(90)), Math.toRadians(170), fastVel, baseAcc)
                .splineToLinearHeading(new Pose2d(-44,7, Math.toRadians(90)), Math.toRadians(170), baseVel, baseAcc)

                .setReversed(true)
                .strafeTo(new Vector2d(-45,49), sonicVel, sonicAcc)

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-55,7), Math.toRadians(-150), fastVel, fastAcc)
                .strafeTo(new Vector2d(-55,49), sonicVel, sonicAcc)

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-64,7), Math.toRadians(-145), fastVel, fastAcc)
                .strafeTo(new Vector2d(-64.5,49), sonicVel, sonicAcc)
                //.waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder moveToPickup2 = pushBlocks1.fresh()
                .strafeTo(new Vector2d(-64.5, 43), baseVel, baseAcc)
                // Move to wall pickup
                .strafeTo(new Vector2d(-64.5, pp), baseVel, baseAcc)
                //.waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder moveToClip2 = moveToPickup2.fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-4, 32), Math.toRadians(270), fastVel, fastAcc)
                //.waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder moveToPickup3 = moveToClip2.fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-48, pp), Math.toRadians(90), baseVel, baseAcc)
                //.waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder moveToClip3 = moveToPickup3.fresh()
                //.waitSeconds(1)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-2, 32), Math.toRadians(270), baseVel, baseAcc)
                .endTrajectory();

        TrajectoryActionBuilder moveToPickup4 = moveToClip3.fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-48, pp), Math.toRadians(90), baseVel, baseAcc)
                .endTrajectory();

        TrajectoryActionBuilder moveToClip4 = moveToPickup4.fresh()
                //.waitSeconds(1)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0, 32), Math.toRadians(270), baseVel, baseAcc)
                .endTrajectory();

        TrajectoryActionBuilder moveToPickup5 = moveToClip4.fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-48, pp), Math.toRadians(90), baseVel, baseAcc)
                .endTrajectory();

        TrajectoryActionBuilder moveToClip5 = moveToPickup5.fresh()
                //.waitSeconds(1)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(2, 32), Math.toRadians(270), baseVel, baseAcc)
                .endTrajectory();

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

        // Init claw Zero extension, then angle
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                actionControl.setServoAnglePosition(0.0).getAction(),
                                actionControl.clawClose().getAction(),
                                actionControl.setSwingPosition(0.0).getAction(),
                                actionControl.lightOff().getAction()
                        ),
                        actionControl.retainAnglePosition().getAction(),
                        actionControl.zeroExtension().getAction(),
                        actionControl.zeroAngle().getAction(),
                        actionControl.zeroExtension().getAction()
                ));

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Init Position");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        clip1Action,
                                        new SequentialAction(
                                                actionControl.prepareClip().getAction(),
                                                new SleepAction(2)
                                        )
                                ),
                                actionControl.clipClip().getAction(),

                                new ParallelAction(
                                        pushBlocks1Action,
                                        actionControl.rest().getAction()
                                ),

                                new ParallelAction(
                                        moveToPickup2Action,
                                        actionControl.wallPickup().getAction()
                                ),
                                actionControl.clawClose().getAction(),
                                new ParallelAction(
                                        moveToClip2Action,
                                        actionControl.prepareClip().getAction()
                                ),
                                actionControl.clipClip().getAction(),

                                new ParallelAction(
                                        moveToPickup3Action,
                                        actionControl.wallPickup().getAction()
                                ),
                                actionControl.clawClose().getAction(),
                                new ParallelAction(
                                        moveToClip3Action,
                                        actionControl.prepareClip().getAction()
                                ),
                                actionControl.clipClip().getAction(),

                                new ParallelAction(
                                        moveToPickup4Action,
                                        actionControl.wallPickup().getAction()
                                ),
                                actionControl.clawClose().getAction(),
                                new ParallelAction(
                                        moveToClip4Action,
                                        actionControl.prepareClip().getAction()
                                ),
                                actionControl.clipClip().getAction(),

                                new ParallelAction(
                                        moveToPickup5Action,
                                        actionControl.wallPickup().getAction()
                                ),
                                actionControl.clawClose().getAction(),
                                new ParallelAction(
                                        moveToClip5Action,
                                        actionControl.prepareClip().getAction()
                                ),
                                actionControl.clipClip().getAction()
                        ), new InstantAction(drive.localizer::update)
                )
        );
    }
}