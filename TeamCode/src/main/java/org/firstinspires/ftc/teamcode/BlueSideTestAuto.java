package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.pipeline.RedProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.nonRR.States;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class BlueSideTestAuto extends SkeletonWithActions {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.currentState = States.INITIAL;

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


        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(
                new SequentialAction(
                        robot.zeroAngle(),
                        robot.zeroExtension(),
                        robot.setServoState(robot.currentState),
                        robot.clawGrab()
                )
        );

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Position during Init", robot.currentState);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
        // go to bucket while raising up
                new SequentialAction(
                        new ParallelAction(
                                goToBucket.build(),
                                new SequentialAction(
                                        robot.setExtensionState(States.DROP),
                                        robot.setAngleState(States.DROP)
                                )
                        ),
                        robot.clawOpen()
                )
                );

        new SequentialAction(
                new ParallelAction(
                        goToBar.build(),
                        new SequentialAction(
                                robot.setExtensionState(States.INITIAL),
                                robot.setAngleState(States.INITIAL)
                        )
                ),
                new SequentialAction(
                        robot.setExtensionState(States.DROP),
                        robot.setAngleState(States.DROP)
                )

        );
    }
}