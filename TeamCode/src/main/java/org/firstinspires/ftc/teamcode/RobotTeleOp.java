package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.nonRR.States;
import org.firstinspires.ftc.teamcode.robothardware.Lift;

import java.util.ArrayList;

public class robotTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        drive.getPoseEstimate();
        // vision here that outputs position

        TrajectoryActionBuilder goToBucket = drive.actionBuilder(initialPose)
                .lineToX(56-10)
                .turnTo(Math.toRadians(225))
                .strafeTo(new Vector2d(56-2,56));
        TrajectoryActionBuilder goToBar = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(34,31))
                .turnTo(Math.toRadians(180))
                .strafeTo(new Vector2d(34,10))
                .strafeTo(new Vector2d(24,10));

        while (!isStopRequested() && !opModeIsActive()){
            telemetry.addData("Position during Init", robot.currentState);
            telemetry.update();
        }


        waitForStart();

    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }
}
