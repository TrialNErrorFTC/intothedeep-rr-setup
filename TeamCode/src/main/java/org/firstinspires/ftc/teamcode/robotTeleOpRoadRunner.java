package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.nonRR.States;
import org.firstinspires.ftc.teamcode.robothardware.Lift;

import java.util.ArrayList;
import java.util.List;



public class robotTeleOpRoadRunner extends SkeletonWithArmActions {
    private List<Action> runningActions = new ArrayList<>();
    FtcDashboard dash = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        LiftWithActions lift = new LiftWithActions();

        // vision here that outputs position

        waitForStart();

        Pose2d currentPose = drive.localizer.getPose();
        //using the joysticks move the robot


        while (opModeIsActive()) {
            Actions.runBlocking(lift.setExtensionPosition());
        }
//            drive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x
//                    ),
//                    -gamepad1.right_stick_x
//            ));
//
//            drive.updatePoseEstimate();
//
//            Pose2d pose = drive.localizer.getPose();
//            telemetry.addData("x", pose.position.x);
//            telemetry.addData("y", pose.position.y);
//            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
//            telemetry.update();
//
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.fieldOverlay().setStroke("#3F51B5");
//            Drawing.drawRobot(packet.fieldOverlay(), pose);
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
//
//            List<Action> newActions = new ArrayList<>();
//            for (Action action : runningActions) {
//                action.preview(packet.fieldOverlay());
//                if (action.run(packet)) {
//                    newActions.add(action);
//                }
//            }
//            runningActions = newActions;
//
//            dash.sendTelemetryPacket(packet);
//
//        }

    }
}
