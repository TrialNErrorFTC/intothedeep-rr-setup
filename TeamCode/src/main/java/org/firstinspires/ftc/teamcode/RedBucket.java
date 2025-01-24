//package org.firstinspires.ftc.teamcode;
//
//// RR-specific imports
//import com.acmerobotics.dashboard.config.Config;
//
//// Non-RR imports
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//@Config
//@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
//public class RedBucket extends AutoCommon {
//
//    @Override
//    public void runOpMode() {
//        // initialize the robot
//        Pose2d initialPose = new Pose2d(20, 60, Math.toRadians(270));
//
//        initialize(new Pose2d(0, 0, Math.toRadians(0)));
//        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose)
//                .splineTo(new Vector2d(48, 48), Math.toRadians(45))
//                .splineTo(new Vector2d(38, 25), Math.toRadians(0))
//                .splineTo(new Vector2d(48, 48), Math.toRadians(45))
//                .splineTo(new Vector2d(38, 25), Math.toRadians(0))
//                .splineTo(new Vector2d(48, 48), Math.toRadians(45))
//                .splineTo(new Vector2d(38, 25), Math.toRadians(0));
//        trajectoryFinal = tab.build();
//        Actions.runBlocking(trajectoryFinal);
//    }
//}
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class RedBucket extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-0.04);
        Action trajectoryActionChosen = tab1.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
                )
        );


    }
}