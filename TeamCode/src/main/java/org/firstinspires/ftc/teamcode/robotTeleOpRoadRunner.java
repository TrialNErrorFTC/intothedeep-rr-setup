package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nonRR.States;
import org.firstinspires.ftc.teamcode.robothardware.Lift;

import java.util.ArrayList;
import java.util.List;



@TeleOp
public class robotTeleOpRoadRunner extends SkeletonWithArmActions {
    private List<Action> runningActions = new ArrayList<>();
    FtcDashboard dash = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//        LiftWithActions lift = new LiftWithActions();
//
//        // vision here that outputs position
//
//        waitForStart();
//
//        Pose2d currentPose = drive.localizer.getPose();
//        //using the joysticks move the robot
//
//
//        while (opModeIsActive()) {
//            drive.updatePoseEstimate();
//
//            Pose2d pose = drive.localizer.getPose();
//            telemetry.addData("x", pose.position.x);
//            telemetry.addData("y", pose.position.y);
//            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
//            telemetry.update();
//            //hold back in position
//            Action highClipInit = new SequentialAction(
//                            new ParallelAction(
//                                    lift.retainExtensionPosition(),
//                                    lift.setAnglePosition(200)
//                            ),
//                            new ParallelAction(
//                                    lift.retainAnglePosition(),
//                                    lift.setExtensionPosition(400)
//                            )
//                    );
//            Actions.runBlocking(highClipInit);
//
//
//            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y), -gamepad1.right_stick_x));
//
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.fieldOverlay().setStroke("#3F51B5");
//            Drawing.drawRobot(packet.fieldOverlay(), pose);
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
//
//            if (gamepad1.dpad_up){
//                runningActions.add(lift.manualUp());
//            }
//            if(gamepad1.dpad_down){
//                runningActions.add(lift.manualDown());
//            }
//            if(gamepad1.dpad_right){
//                runningActions.add(lift.manualExtend());
//            }
//            if(gamepad1.dpad_left){
//                runningActions.add(lift.manualRetract());
//            }
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

        @Override
        public void runOpMode() throws InterruptedException {
            Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
            LiftWithActions lift = new LiftWithActions();

            waitForStart();

            while (opModeIsActive()) {
                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y), -gamepad1.right_stick_x));
                drive.updatePoseEstimate();
                if (gamepad1.dpad_up) {
                    runningActions.add(lift.manualUp());
                }
                if (gamepad1.dpad_down) {
                    runningActions.add(lift.manualDown());
                }
                if (gamepad1.dpad_right) {
                    runningActions.add(lift.manualExtend());
                }
                if (gamepad1.dpad_left) {
                    runningActions.add(lift.manualRetract());
                }

                List<Action> newActions = new ArrayList<>();
                for (Action action : runningActions) {
                    if (!action.run(packet)) {
                        newActions.add(action);
                    }
                }
                runningActions = newActions;

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        }
    }

