package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipeline.RedProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;




@TeleOp
public class TeleOpRR extends TeleOpActionsRR {
    private List<ActionControl.Result> runningActions = new ArrayList<>();
    FtcDashboard dash = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    VisionPortal myVisionPortal;
    RedProcessor redProcessor;



    public void addAction(ActionControl.Result result) {
        boolean exists = false;
        for (ActionControl.Result acc : runningActions) {
            if (acc.getName().equals(result.getName())) {
                exists = true;
            }
        }
        if (!exists) {
            runningActions.add(result);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ActionControl actionControl = new ActionControl();
        Pose2d pose;

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
                )
        );
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), redProcessor);


        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();

            pose = drive.localizer.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            actionControl.liftTelemetry(telemetry);


            // Set drive powers from gamepad input
            PoseVelocity2d driveControl = new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x
            );
            drive.setDrivePowers(driveControl);

            // Main Controller actions
            if (gamepad1.square) {
                //runningActions.add(actionControl.pickup());
                addAction(actionControl.pickup());
            }
            if (gamepad1.triangle) {
                //runningActions.add(actionControl.wallPickup());
                addAction(actionControl.wallPickup());
            }
            if (gamepad1.cross) {
                //runningActions.add(actionControl.prepareClip());
                addAction(actionControl.prepareClip());
            }
            if (gamepad1.circle) {
                //runningActions.add(actionControl.clipClip());
                addAction(actionControl.clipClip());
            }
            if (gamepad1.left_bumper) {
                //runningActions.add(actionControl.clawOpen());
                addAction(actionControl.clawOpen());
            }
            if (gamepad1.right_bumper) {
                //runningActions.add(actionControl.clawClose());
                addAction(actionControl.clawClose());
            }
            if (gamepad1.dpad_down) {
                //runningActions.add(actionControl.pickupSample());
                addAction(actionControl.pickupSample());
            }
            if (gamepad1.dpad_up) {
                // TODO : Add a function for killing all actions
            }
            if (gamepad1.dpad_left) {
                //runningActions.add(actionControl.drop());
                addAction(actionControl.drop());
            }
            if (gamepad1.touchpad) {
                addAction(actionControl.rest());
            }

            // Secondary Controller actions
            if (gamepad2.dpad_up) {
                //runningActions.add(actionControl.manualUp());
                addAction(actionControl.manualUp());
            }
            if (gamepad2.dpad_down) {
                //runningActions.add(actionControl.manualDown());
                addAction(actionControl.manualDown());
            }
            if (gamepad2.dpad_right) {
                //runningActions.add(actionControl.manualExtend());
                addAction(actionControl.manualExtend());
            }
            if (gamepad2.dpad_left) {
                //runningActions.add(actionControl.manualRetract());
                addAction(actionControl.manualRetract());
            }
            if (gamepad2.triangle) {
                //runningActions.add(actionControl.manualClawAngle(-0.01));
                addAction(actionControl.manualClawAngle(-0.01));
            }
            if (gamepad2.square) {
                //runningActions.add(actionControl.manualClawAngle(0.01));
                addAction(actionControl.manualClawAngle(0.01));
            } if (gamepad2.cross) {
                //runningActions.add(actionControl.hangPrepare());
                addAction(actionControl.hangPrepare());
            } if (gamepad2.circle) {
                //runningActions.add(actionControl.hang());
                addAction(actionControl.hang());
            }

            // Filter duplicates
//            Set<Action> set = new HashSet<ActionControl.Result>(runningActions);
//            runningActions.clear();
//            runningActions.addAll(set);
//            telemetry.addData("set actions", set.size());
//            telemetry.addData("running actions", runningActions.size());

            // Ensure actions to retain positions are always running
            runningActions.add(actionControl.result (
                    new ParallelAction(
                        actionControl.retainAnglePosition().getAction(),
                        actionControl.retainExtensionPosition().getAction()
                    ),
                    "hold"));

            // Prepare the list of actions, including the drive control
            List<ActionControl.Result> newActions = new ArrayList<>();
            for (ActionControl.Result action : runningActions) {
                if (action.getAction().run(packet)) {
                    newActions.add(action);
                }
//                newActions.add(new ParallelAction(
//                        action
//                        //actionControl.driveControl(drive)  // Ensure drive control runs parallel to other actions
//                ));
            }
            runningActions = newActions;

            telemetry.update();
//            // This is where the fix is: we only run actions in parallel with the drive controls without blocking them
//            Actions.runBlocking(
//                    new ParallelAction(
//                            newActions.toArray(new Action[0])  // Execute everything in parallel
//                    )
//            );
//
//            // Ensure actions to retain positions are always running
//            Actions.runBlocking(
//
//            );



            // Send telemetry packet to dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
