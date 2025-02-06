package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Autonomous
public class RedSidePreload extends TeleOpActionsRR {
    FtcDashboard dash = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-22+8,72-8  , Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ActionControl actionControl = new ActionControl();
        Pose2d pose;


        // vision here that outputs position

        //All TrajectoryActionsBuilders here
        //DONE: Set the drive positions to
        TrajectoryActionBuilder moveToClipLocation = drive.actionBuilder(initialPose)
                .lineToY(56)
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(3, 38))
                .strafeTo(new Vector2d(3, 30));
//                .turnTo(Math.toRadians(270));
//        TrajectoryActionBuilder moveToClipLocationAdjust = drive.actionBuilder(initialPose)
//                .strafeTo(new Vector2d(0, 20 + 9));
//        TrajectoryActionBuilder moveToWallArea = drive.actionBuilder(initialPose)
//                .strafeTo(new Vector2d(-52, 53));
        TrajectoryActionBuilder moveToParkPosition = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-52, 60))
                .turnTo(Math.toRadians(270));


        //DONE: set all Actions here
        Action preloadClip = new SequentialAction(
                //move to clip location & position and prepareClip
                new ParallelAction(
                        moveToClipLocation.build(),
                        actionControl.prepareClip().getAction()
                ),
                //clip the position
                actionControl.clipClip().getAction(),
                actionControl.rest().getAction()
        );

//        Action humanPlayerClip = new SequentialAction(
//                //move to wall area and wall pickup
//                new ParallelAction(
//                        moveToWallArea.build(),
//                        actionControl.wallPickup().getAction()
//                ),
//                //close claw
//                actionControl.clawClose().getAction(),
//                //move to clip location & position (adjustable), and prepare clip
//                new ParallelAction(
//                        moveToClipLocationAdjust.build(),
//                        actionControl.prepareClip().getAction()
//                ),
//                //clip the position
//                actionControl.clipClip().getAction()
//        );
//
        Action park = new ParallelAction(
                moveToParkPosition.build(),
                actionControl.rest().getAction()
        );
        // actions that need to happen on init; for instance, a claw tightening.



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
                                preloadClip,
                                park
                        ), new InstantAction(drive.localizer::update)
                )
        );

    }
}
