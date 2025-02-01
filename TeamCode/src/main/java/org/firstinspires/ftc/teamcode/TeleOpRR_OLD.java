//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import java.util.ArrayList;
//import java.util.List;
//
//
//
//@TeleOp
//public class TeleOpRR_OLD extends TeleOpActionsRR {
//    private List<Action> runningActions = new ArrayList<>();
//    FtcDashboard dash = FtcDashboard.getInstance();
//    TelemetryPacket packet = new TelemetryPacket();
//
////    @Override
////    public void runOpMode() throws InterruptedException {
////        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
////        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
////        LiftWithActions lift = new LiftWithActions();
////
////        // vision here that outputs position
////
////        waitForStart();
////
////        Pose2d currentPose = drive.localizer.getPose();
////        //using the joysticks move the robot
////
////
////        while (opModeIsActive()) {
////            drive.updatePoseEstimate();
////
////            Pose2d pose = drive.localizer.getPose();
////            telemetry.addData("x", pose.position.x);
////            telemetry.addData("y", pose.position.y);
////            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
////            telemetry.update();
////            //hold back in position
////            Action highClipInit = new SequentialAction(
////                            new ParallelAction(
////                                    lift.retainExtensionPosition(),
////                                    lift.setAnglePosition(200)
////                            ),
////                            new ParallelAction(
////                                    lift.retainAnglePosition(),
////                                    lift.setExtensionPosition(400)
////                            )
////                    );
////            Actions.runBlocking(highClipInit);
////
////
////            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y), -gamepad1.right_stick_x));
////
////            TelemetryPacket packet = new TelemetryPacket();
////            packet.fieldOverlay().setStroke("#3F51B5");
////            Drawing.drawRobot(packet.fieldOverlay(), pose);
////            FtcDashboard.getInstance().sendTelemetryPacket(packet);
////
////            if (gamepad1.dpad_up){
////                runningActions.add(lift.manualUp());
////            }
////            if(gamepad1.dpad_down){
////                runningActions.add(lift.manualDown());
////            }
////            if(gamepad1.dpad_right){
////                runningActions.add(lift.manualExtend());
////            }
////            if(gamepad1.dpad_left){
////                runningActions.add(lift.manualRetract());
////            }
////
////            List<Action> newActions = new ArrayList<>();
////            for (Action action : runningActions) {
////                action.preview(packet.fieldOverlay());
////                if (action.run(packet)) {
////                    newActions.add(action);
////                }
////            }
////            runningActions = newActions;
////
////            dash.sendTelemetryPacket(packet);
////
////        }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//        ActionControl actionControl = new ActionControl();
//        Pose2d pose;
//
//        // Init claw Zero extension, then angle
//        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                actionControl.clawClose(),
//                                actionControl.setSwingPosition(0.0)
//                        ),
//                        actionControl.retainAnglePosition(),
//                        actionControl.zeroExtension(),
//                        actionControl.zeroAngle(),
//                        actionControl.zeroExtension()
//                )
//        );
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            drive.updatePoseEstimate();
//
//            pose = drive.localizer.getPose();
//            telemetry.addData("x", pose.position.x);
//            telemetry.addData("y", pose.position.y);
//            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
//            actionControl.liftTelemetry(telemetry);
//            telemetry.update();
//
//            // Set drive powers from gamepad input
//            drive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
//                    -gamepad1.right_stick_x));
//
//            // Main Controller actions
//            if (gamepad1.square) {
//                runningActions.add(actionControl.pickup());
//            }
//            if (gamepad1.triangle) {
//                runningActions.add(actionControl.wallPickup());
//            }
//            if (gamepad1.cross) {
//                runningActions.add(actionControl.prepareClip());
//            }
//            if (gamepad1.circle) {
//                runningActions.add(actionControl.clipClip());
//            }
//            if (gamepad1.left_bumper) {
//                runningActions.add(actionControl.clawOpen());
//            }
//            if (gamepad1.right_bumper) {
//                runningActions.add(actionControl.clawClose());
//            }
//            if (gamepad1.dpad_down) {
//                runningActions.add(actionControl.pickupSample());
//            }
//            if (gamepad1.dpad_up) {
//                // TODO : Add a function for killing all actions
//            }
//
//            // Secondary Controller actions
//            if (gamepad2.dpad_up) {
//                runningActions.add(actionControl.manualUp());
//            }
//            if (gamepad2.dpad_down) {
//                runningActions.add(actionControl.manualDown());
//            }
//            if (gamepad2.dpad_right) {
//                runningActions.add(actionControl.manualExtend());
//            }
//            if (gamepad2.dpad_left) {
//                runningActions.add(actionControl.manualRetract());
//            }
//            if (gamepad2.triangle) {
//                runningActions.add(actionControl.manualClawAngle(-0.01));
//            }
//            if (gamepad2.square) {
//                runningActions.add(actionControl.manualClawAngle(0.01));
//            } if (gamepad2.cross) {
//                runningActions.add(actionControl.hangPrepare());
//            } if (gamepad2.circle) {
//                runningActions.add(actionControl.hang());
//            }
//
//            // Prepare the list of actions, including the drive control
//            List<Action> newActions = new ArrayList<>();
//            for (Action action : runningActions) {
////                driveControl = new PoseVelocity2d(
////                        new Vector2d(0, 0),
////                        0
////                );
////                drive.setDrivePowers(driveControl);
//                newActions.add(new ParallelAction(
//                        actionControl.driveControl(drive, action)
//                        //actionControl.driveControl(drive)  // Ensure drive control runs parallel to other actions
//                ));
//            }
//
//            // This is where the fix is: we only run actions in parallel with the drive controls without blocking them
//            Actions.runBlocking(
//                    new ParallelAction(
//                            newActions.toArray(new Action[0])  // Execute everything in parallel
//                    )
//            );
//
//            // Ensure actions to retain positions are always running
//            Actions.runBlocking(
//                    new ParallelAction(
//                            actionControl.retainAnglePosition(),
//                            actionControl.retainExtensionPosition()
//                    )
//            );
//
//            runningActions = newActions;
//
//            // Send telemetry packet to dashboard
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.fieldOverlay().setStroke("#3F51B5");
//            Drawing.drawRobot(packet.fieldOverlay(), pose);
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
//        }
//    }
//}
