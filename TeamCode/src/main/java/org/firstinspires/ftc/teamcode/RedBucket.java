package org.firstinspires.ftc.teamcode;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class RedBucket extends AutoCommon {

    @Override
    public void runOpMode() {
        // initialize the robot
        Pose2d initialPose = new Pose2d(20, 60, Math.toRadians(270));

        initialize(new Pose2d(0, 0, Math.toRadians(0)));
        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(48, 48), Math.toRadians(45))
                .splineTo(new Vector2d(38, 25), Math.toRadians(0))
                .splineTo(new Vector2d(48, 48), Math.toRadians(45))
                .splineTo(new Vector2d(38, 25), Math.toRadians(0))
                .splineTo(new Vector2d(48, 48), Math.toRadians(45))
                .splineTo(new Vector2d(38, 25), Math.toRadians(0));
        trajectoryFinal = tab.build();
        Actions.runBlocking(trajectoryFinal);
    }
}
