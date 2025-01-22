package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.nonRR.States;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public abstract class AutoCommon extends LinearOpMode {
    public static int EXTEND_INIT_POS = 0;
    public static int EXTEND_INTAKE_POS = 0;
    public static int EXTEND_HOLD_POS = 0;
    public static int EXTEND_LOW_POLE_POS = 125;
    public static int EXTEND_HIGH_POLE_POS = 250;
    public static int EXTEND_LOW_BUCKET_POS = 200;
    public static int EXTEND_HIGH_BUCKET_POS = 1000;

    MecanumDrive drive;
    Claw claw;
    Lift lift;
    Action trajectoryFinal;



    public static int ANGLE_INIT_POS = 0;
    public static int ANGLE_INTAKE_POS = 0;
    public static int ANGLE_HOLD_POS = 0;
    public static int ANGLE_LOW_POLE_POS = 0;
    public static int ANGLE_HIGH_POLE_POS = 0;
    public static int ANGLE_LOW_BUCKET_POS = 0;
    public static int ANGLE_HIGH_BUCKET_POS = 0;

    public States state = States.INIT;
    public class Lift {
        public DcMotor motorAngle1;
        public DcMotor motorAngle2;
        public DcMotor motorExtension1;
        public DcMotor motorExtension2;
        public DcMotor[] liftMotors;
    

        public Lift(HardwareMap hardwareMap) {
            motorAngle1 = hardwareMap.get(DcMotor.class, "motorAngle1");


            motorAngle1.setDirection(DcMotorSimple.Direction.FORWARD);
            motorAngle1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorAngle1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorAngle1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            motorAngle2 = hardwareMap.get(DcMotor.class, "motorAngle2");


            motorAngle2.setDirection(DcMotorSimple.Direction.REVERSE);
            motorAngle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorAngle2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorAngle2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            motorExtension1 = hardwareMap.get(DcMotor.class, "motorExtension1");


            motorExtension1.setDirection(DcMotorSimple.Direction.REVERSE);
            motorExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorExtension1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorExtension1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            motorExtension2 = hardwareMap.get(DcMotor.class, "motorExtension2");


            motorExtension2.setDirection(DcMotorSimple.Direction.FORWARD);
            motorExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorExtension2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorExtension2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotors = new DcMotor[]{motorAngle1, motorAngle2, motorExtension1, motorExtension2};

        }
        public class moveToState implements Action{
            boolean initalized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket Packet){
                while (!initalized){
                    motorAngle1.setTargetPosition(state.motorAnglePosition);
                    motorAngle2.setTargetPosition(state.motorAnglePosition);
                    motorExtension1.setTargetPosition(state.motorExtensionPosition);
                    motorExtension2.setTargetPosition(state.motorExtensionPosition);

                    motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    motorAngle1.setPower(0.5);
                    motorAngle2.setPower(0.5);
                    motorExtension1.setPower(0.5);
                    motorExtension2.setPower(0.5);
                    telemetry.addData("Arm State", state);
                    }
                return false;
            }
        }

        public class telemetryUpdate implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                    telemetry.addData("extension1 pos", motorExtension1.getCurrentPosition());
                    telemetry.addData("extension2 pos", motorExtension2.getCurrentPosition());
                    telemetry.addData("angle1 pos", motorAngle1.getCurrentPosition());
                    telemetry.addData("angle2 pos", motorAngle2.getCurrentPosition());
                return false;
            }
        }

    }
    public void telemetryUpdate(Telemetry telemetry) {
        telemetry.addData("trigger", gamepad1.left_trigger);
        telemetry.update();
    }


    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.55);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }
    public void initialize(Pose2d initialPose) {
        drive = new MecanumDrive(hardwareMap, initialPose);
        claw = new Claw(hardwareMap);
        lift = new Lift(hardwareMap);
    }
    
    public abstract void runOpMode();
    
//    @Override
//    public void runOpMode() {
//        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//        Claw claw = new Claw(hardwareMap);
//        Lift lift = new Lift(hardwareMap);
//
//        // vision here that outputs position
//        int visionOutputPosition = 1;
//
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(33, Math.toRadians(0))
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .setTangent(Math.toRadians(0))
//                .lineToX(32)
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3);
//        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
//                .lineToY(37)
//                .setTangent(Math.toRadians(0))
//                .lineToX(18)
//                .waitSeconds(3)
//                .setTangent(Math.toRadians(0))
//                .lineToXSplineHeading(46, Math.toRadians(180))
//                .waitSeconds(3);
//        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(33, Math.toRadians(180))
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(46, 30))
//                .waitSeconds(3);
//        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
//                .strafeTo(new Vector2d(48, 12))
//                .build();
//
//        // actions that need to happen on init; for instance, a claw tightening.
//        Actions.runBlocking(claw.closeClaw());
//
//
//        while (!isStopRequested() && !opModeIsActive()) {
//            int position = visionOutputPosition;
//            telemetry.addData("Position during Init", position);
//            telemetry.update();
//        }
//
//        int startPosition = visionOutputPosition;
//        telemetry.addData("Starting Position", startPosition);
//        telemetry.update();
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        Action trajectoryActionChosen;
//        if (startPosition == 1) {
//            trajectoryActionChosen = tab1.build();
//        } else if (startPosition == 2) {
//            trajectoryActionChosen = tab2.build();
//        } else {
//            trajectoryActionChosen = tab3.build();
//        }
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        trajectoryActionChosen,
//                        lift.liftUp(),
//                        claw.openClaw(),
//                        lift.liftDown(),
//                        trajectoryActionCloseOut
//                )
//        );
    }
