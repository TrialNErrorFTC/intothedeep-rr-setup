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
public class BlueSideTestAuto extends LinearOpMode {

    public class RobotHardware{
        public static final double DRIVE_MOTOR_TICKS_PER_ROTATION = 384.5;
        public static final double WHEEL_DIAMETER = 96 / 25.4;

        public DcMotor frontLeft = null;
        public DcMotor frontRight = null;
        public DcMotor rearLeft = null;
        public DcMotor rearRight = null;
        public DcMotor[] drivetrainMotors = null;
        public DcMotor[] liftMotors = null;
        public DcMotor motorAngle1;
        public DcMotor motorAngle2;
        public DcMotor motorExtension1;
        public DcMotor motorExtension2;
        public Servo claw;
        public Servo swing;
        public Servo angle;
        public TouchSensor limitSwitchAngle;
        public TouchSensor limitSwitchExtension;
        public BNO055IMU imu;
        public States currentState;
        public boolean servoAngleInitialized;
        public double currentAngle;
        double k = 1;
        int tickConversionConstant = (int) (751.8 / 537.7);
        TouchSensor setup_touch;
        VisionPortal myVisionPortal;
        RedProcessor redProcessor = new RedProcessor();

        public RobotHardware(HardwareMap hardwareMap) {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            rearLeft = hardwareMap.get(DcMotor.class, "backLeft");
            rearRight = hardwareMap.get(DcMotor.class, "backRight");
            drivetrainMotors = new DcMotor[]{frontLeft, frontRight, rearLeft, rearRight};

            for (DcMotor motor : drivetrainMotors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

            motorAngle1 = hardwareMap.get(DcMotor.class, "motorAngle1");
            motorAngle1.setDirection(DcMotorSimple.Direction.FORWARD);
            motorAngle1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorAngle1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorAngle2 = hardwareMap.get(DcMotor.class, "motorAngle2");
            motorAngle2.setDirection(DcMotorSimple.Direction.REVERSE);
            motorAngle2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorAngle2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorExtension1 = hardwareMap.get(DcMotor.class, "motorExtension1");
            motorExtension1.setDirection(DcMotorSimple.Direction.REVERSE);
            motorExtension1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorExtension1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorExtension2 = hardwareMap.get(DcMotor.class, "motorExtension2");
            motorExtension2.setDirection(DcMotorSimple.Direction.FORWARD);
            motorExtension2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorExtension2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            liftMotors = new DcMotor[]{motorAngle1, motorAngle2, motorExtension1, motorExtension2};

            claw = hardwareMap.servo.get("claw");
            swing = hardwareMap.servo.get("swing");
            angle = hardwareMap.servo.get("angle");

            limitSwitchAngle = hardwareMap.touchSensor.get("limitAngle");
            limitSwitchExtension = hardwareMap.touchSensor.get("limitExtension");

            initializeIMU(hardwareMap);
        }

        private void initializeIMU(HardwareMap hardwareMap) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitImuCalibration.json";
            parameters.loggingEnabled = false;
            parameters.accelerationIntegrationAlgorithm = null;

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }

        public void startMove(double drive, double strafe, double turn, double scale) {
            double powerFL = (drive + strafe + turn) * scale;
            double powerFR = (drive - strafe - turn) * scale;
            double powerBL = (drive - strafe + turn) * scale;
            double powerBR = (drive + strafe - turn) * scale;

            double maxPower = Math.max(Math.max(Math.abs(powerFL), Math.abs(powerFR)), Math.max(Math.abs(powerBL), Math.abs(powerBR)));
            double max = (maxPower < 1) ? 1 : maxPower;

            frontLeft.setPower(Range.clip(powerFL / max, -1, 1));
            frontRight.setPower(Range.clip(powerFR / max, -1, 1));
            rearLeft.setPower(Range.clip(powerBL / max, -1, 1));
            rearRight.setPower(Range.clip(powerBR / max, -1, 1));
        }

        public void stopMove() {
            for (DcMotor motor : drivetrainMotors) {
                motor.setPower(0);
            }
            motorAngle1.setPower(0);
            motorAngle2.setPower(0);
            motorExtension1.setPower(0);
            motorExtension2.setPower(0);
        }

        public class SetExtensionState implements Action {
            private final States state;
            private boolean initialized = false;

            public SetExtensionState(States state) {
                this.state = state;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motorExtension1.setTargetPosition(state.motorExtensionPosition);
                    motorExtension2.setTargetPosition(state.motorExtensionPosition);

                    motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    motorExtension1.setPower(0.5);
                    motorExtension2.setPower(0.5);
                    initialized = true;
                }

                packet.put("extension1 pos", motorExtension1.getCurrentPosition());
                packet.put("extension2 pos", motorExtension2.getCurrentPosition());

                return !motorExtension1.isBusy() && !motorExtension2.isBusy();
            }
        }

        public Action setExtensionState(States state) {
            return new SetExtensionState(state);
        }

        public class SetServoState implements Action {
            private final States state;

            public SetServoState(States state) {
                this.state = state;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                swing.setPosition(state.swingPosition);
                angle.setPosition(state.anglePosition);
                packet.put("swing pos", swing.getPosition());
                packet.put("angle pos", angle.getPosition());
                return false;
            }
        }

        public Action setServoState(States state) {
            return new SetServoState(state);
        }

        public class SetAngleState implements Action {
            private final States state;
            private boolean initialized = false;

            public SetAngleState(States state) {
                this.state = state;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motorAngle1.setTargetPosition(state.motorAnglePosition);
                    motorAngle2.setTargetPosition(state.motorAnglePosition);

                    motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    motorAngle1.setPower(0.25);
                    motorAngle2.setPower(0.25);
                    initialized = true;
                }

                packet.put("angle1 pos", motorAngle1.getCurrentPosition());
                packet.put("angle2 pos", motorAngle2.getCurrentPosition());

                return !motorAngle1.isBusy() && !motorAngle2.isBusy();
            }
        }

        public Action setAngleState(States state) {
            return new SetAngleState(state);
        }

        public class ClawOpen implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.5);
                packet.put("claw pos", claw.getPosition());
                return false;
            }
        }

        public Action clawOpen() {
            return new ClawOpen();
        }

        public class ClawGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.35);
                packet.put("claw pos", claw.getPosition());
                return false;
            }
        }

        public Action clawGrab() {
            return new ClawGrab();
        }

        public class ZeroExtension implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gamepad1.setLedColor(255, 0, 2500, 2500);
                boolean lowered = false;
                while (true) {
                    if (limitSwitchExtension.isPressed()) {
                        lowered = true;
                        break;
                    }
                    motorExtension1.setTargetPosition(motorExtension1.getCurrentPosition() + 5);
                    motorExtension2.setTargetPosition(motorExtension2.getCurrentPosition() + 5);
                    motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorExtension1.setPower(0.5);
                    motorExtension2.setPower(0.5);

                    packet.put("extension1 pos", motorExtension1.getCurrentPosition());
                    packet.put("extension2 pos", motorExtension2.getCurrentPosition());
                    packet.put("angle1 pos", motorAngle1.getCurrentPosition());
                    packet.put("angle2 pos", motorAngle2.getCurrentPosition());
                }
                if (lowered) {
                    motorExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    packet.put("extension1 pos", motorExtension1.getCurrentPosition());
                    packet.put("extension2 pos", motorExtension2.getCurrentPosition());
                    packet.put("angle1 pos", motorExtension1.getCurrentPosition());
                    packet.put("angle2 pos", motorExtension2.getCurrentPosition());
                }
                return false;
            }
        }

        public Action zeroExtension() {
            return new ZeroExtension();
        }

        public class ZeroAngle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                boolean lowered = false;
                while (true) {
                    if (limitSwitchAngle.isPressed()) {
                        lowered = true;
                        break;
                    }
                    motorAngle1.setTargetPosition(motorAngle1.getCurrentPosition() - 5);
                    motorAngle2.setTargetPosition(motorAngle2.getCurrentPosition() - 5);
                    motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorAngle1.setPower(0.5);
                    motorAngle2.setPower(0.5);

                    packet.put("extension1 pos", motorExtension1.getCurrentPosition());
                    packet.put("extension2 pos", motorExtension2.getCurrentPosition());
                    packet.put("angle1 pos", motorAngle1.getCurrentPosition());
                    packet.put("angle2 pos", motorAngle2.getCurrentPosition());
                }
                if (lowered) {
                    motorAngle1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorAngle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    packet.put("extension1 pos", motorExtension1.getCurrentPosition());
                    packet.put("extension2 pos", motorExtension2.getCurrentPosition());
                    packet.put("angle1 pos", motorAngle1.getCurrentPosition());
                    packet.put("angle2 pos", motorAngle2.getCurrentPosition());
                }
                return false;
            }
        }

        public Action zeroAngle() {
            return new ZeroAngle();
        }
    }

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