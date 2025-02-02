package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class RedSidePreload extends TeleOpActionsRR {
    FtcDashboard dash = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(8, -70+8, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ActionControl actionControl = new ActionControl();
        Pose2d pose;

        // vision here that outputs position

        TrajectoryActionBuilder preloadClip = drive.actionBuilder(initialPose).lineToY(24+8);
        TrajectoryActionBuilder goToPark = drive.actionBuilder(initialPose).strafeTo(new Vector2d(55, -70+8));

        // actions that need to happen on init; for instance, a claw tightening.

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Init Position");
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        actionControl.prepareClip().getAction(),
                        preloadClip.build(),
                        actionControl.clipClip().getAction(),
                        goToPark.build()
                )
        );

    }
}