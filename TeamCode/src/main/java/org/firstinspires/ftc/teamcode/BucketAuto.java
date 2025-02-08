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
public class BucketAuto extends TeleOpActionsRR {
    FtcDashboard dash = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-22+13.5,72-10, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ActionControl actionControl = new ActionControl();

        TrajectoryActionBuilder preclipDropPosition = drive.actionBuilder(new Pose2d(14, 62, Math.toRadians(180)))
                .lineToX(56-8)
                .endTrajectory();

        TrajectoryActionBuilder goToFirstBlock = preclipDropPosition.fresh()
                .splineToLinearHeading(new Pose2d(new Vector2d(47, 39), Math.toRadians(270)), Math.toRadians(270))
                .endTrajectory();

        TrajectoryActionBuilder goToRegularDropPosition = goToFirstBlock.fresh()
                .setReversed(true)
                .splineTo(new Vector2d(56, 57), Math.toRadians(45))
                .endTrajectory();

        TrajectoryActionBuilder goToSecondBlock = goToRegularDropPosition.fresh()
                .setReversed(false)
                .splineTo(new Vector2d(58, 39), Math.toRadians(270))
                .endTrajectory();


        Action preclipDropAction = new ParallelAction(
                preclipDropPosition.build(),
                actionControl.drop().getAction()
        );
        Action goToFirstBlockAction = new ParallelAction(
                goToFirstBlock.build(),
                actionControl.setServoAnglePosition(90/300).getAction()
        );
        Action regularDropAction = goToRegularDropPosition.build();
        Action goToSecondBlockAction = goToSecondBlock.build();

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
    }
}