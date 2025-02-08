package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.net.wifi.p2p.WifiP2pManager;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.nonRR.States;
import org.firstinspires.ftc.teamcode.robothardware.ControlRR;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public abstract class TeleOpActionsRR extends LinearOpMode {
    ControlRR controlRR;
    private double extensionPower= 0.65;
    private double anglePower = 0.5;
    private double extensionHoldPower = 0.5;
    private double angleHoldPower = 0.6;
    private int extensionTolerance = 3;
    private int angleTolerance = 4;

    // Store current motor value targets
    public int currentExtensionPosition = 0;
    public int currentAnglePosition = 0;
    private int angleIncrement = 15;
    private int extensionIncrement = 28;
    private double extendRatio = 28.0 / 42;

    private RedProcessor.Alignment alignment;

    public class ActionControl {
        public ActionControl(){
            controlRR = new ControlRR(hardwareMap);
        }

        public class Result {
            String name;
            Action action;

            public Result(Action acc, String n) {
                this.action = acc;
                this.name = n;
            }

            public Action getAction() {
                return action;
            }

            public String getName() {
                return name;
            }
        }

        public Result result(Action acc, String n) {
            return new Result(acc, n);
        }

        public void liftTelemetry(Telemetry t) {
            t.addData("motorExtension1", controlRR.motorExtension1.getCurrentPosition());
            t.addData("motorExtension2", controlRR.motorExtension1.getCurrentPosition());
            t.addData("motorAngle1", controlRR.motorAngle1.getCurrentPosition());
            t.addData("motorAngle2", controlRR.motorAngle2.getCurrentPosition());
            t.addData("currentExtensionPosition", currentExtensionPosition);
            t.addData("currentAnglePosition", currentAnglePosition);
        }

        public class setExtensionPosition implements Action {
            private final int extensionPosition;
            private boolean initialized = false;

            public setExtensionPosition(int extensionPosition) {
                this.extensionPosition = extensionPosition;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                currentExtensionPosition = extensionPosition;

                // keep the motor in a position that is in the tolerance range, and ends when going to the next action
                controlRR.motorAngle1.setTargetPosition(currentAnglePosition);
                controlRR.motorAngle2.setTargetPosition(currentAnglePosition);

                controlRR.motorAngle1.setTargetPositionTolerance(angleTolerance);
                controlRR.motorAngle2.setTargetPositionTolerance(angleTolerance);

                controlRR.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                controlRR.motorAngle1.setPower(angleHoldPower);
                controlRR.motorAngle2.setPower(angleHoldPower);

                //set position of lift
                controlRR.motorExtension1.setTargetPosition(extensionPosition);
                controlRR.motorExtension2.setTargetPosition(extensionPosition);

                //set mode to run to position
                controlRR.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                controlRR.motorExtension1.setPower(extensionPower);
                controlRR.motorExtension2.setPower(extensionPower);

                if (Math.abs(controlRR.motorExtension1.getCurrentPosition() - extensionPosition) < extensionTolerance*2){
                    return false;
                } else {
                    return true;
                }
            };
        }
        public Result setExtensionPosition(int position) {
            return new Result(new setExtensionPosition(position), "setExtensionPosition");
        }

        public class SetAnglePosition implements Action {
            private boolean initialized = false;
            private int anglePosition;

            public SetAnglePosition(int anglePosition) {
                this.anglePosition = anglePosition;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                currentExtensionPosition -= (int) ((anglePosition-currentAnglePosition) * extendRatio);
                currentAnglePosition = anglePosition;

                //set position of lift
                controlRR.motorExtension1.setTargetPosition(currentExtensionPosition);
                controlRR.motorExtension2.setTargetPosition(currentExtensionPosition);

                controlRR.motorExtension1.setTargetPositionTolerance(extensionTolerance);
                controlRR.motorExtension2.setTargetPositionTolerance(extensionTolerance);

                //set mode to run to position
                controlRR.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                controlRR.motorExtension1.setPower(extensionPower);
                controlRR.motorExtension2.setPower(extensionPower);


                //set position of lift
                controlRR.motorAngle1.setTargetPosition(anglePosition);
                controlRR.motorAngle2.setTargetPosition(anglePosition);

                //set mode to run to position
                controlRR.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                controlRR.motorAngle1.setPower(anglePower);
                controlRR.motorAngle2.setPower(anglePower);

                if (Math.abs(controlRR.motorAngle2.getCurrentPosition() - anglePosition) < angleTolerance || (controlRR.motorAngle2.getCurrentPosition() - anglePosition) < angleTolerance){
                    return false;
                } else {
                    return true;
                }
            };
        }
        public Result setAnglePosition(int position) {
            return new Result(new SetAnglePosition(position), "setAnglePosition");
        }

        public class retainExtensionPosition implements Action {
            private boolean initialized = false;
            private int currentPosition1;
            private int currentPosition2;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized){
                    initialized = true;
                }

                // keep the motor in a position that is in the tolerance range, and ends when going to the next action
                controlRR.motorExtension1.setTargetPosition(currentExtensionPosition);
                controlRR.motorExtension2.setTargetPosition(currentExtensionPosition);
                controlRR.motorExtension1.setTargetPositionTolerance(extensionTolerance);
                controlRR.motorExtension2.setTargetPositionTolerance(extensionTolerance);
                controlRR.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                controlRR.motorExtension1.setPower(extensionHoldPower);
                controlRR.motorExtension2.setPower(extensionHoldPower);

                return false;
            }
        }
        public Result retainExtensionPosition() {
            return new Result(new retainExtensionPosition(), "retainExtensionPosition");
        }

        public class retainAnglePosition implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // keep the motor in a position that is in the tolerance range, and ends when going to the next action
                controlRR.motorAngle1.setTargetPosition(currentAnglePosition);
                controlRR.motorAngle2.setTargetPosition(currentAnglePosition);

                controlRR.motorAngle1.setTargetPositionTolerance(angleTolerance);
                controlRR.motorAngle2.setTargetPositionTolerance(angleTolerance);

                controlRR.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                controlRR.motorAngle1.setPower(angleHoldPower);
                controlRR.motorAngle2.setPower(angleHoldPower);

                return false;
            }
        }
        public Result retainAnglePosition() {
            return new Result(new retainAnglePosition(), "retainAnglePosition");
        }

        public class manualUp implements Action {
            boolean initialized = false;
            int additivePosition1;
            int additivePosition2;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    currentAnglePosition += angleIncrement;
                    initialized = true;
                }

                currentExtensionPosition -= (int) (angleIncrement * extendRatio);

                //set position of lift
                controlRR.motorExtension1.setTargetPosition(currentExtensionPosition);
                controlRR.motorExtension2.setTargetPosition(currentExtensionPosition);

                controlRR.motorExtension1.setTargetPositionTolerance(extensionTolerance);
                controlRR.motorExtension2.setTargetPositionTolerance(extensionTolerance);

                //set mode to run to position
                controlRR.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                controlRR.motorExtension1.setPower(extensionPower);
                controlRR.motorExtension2.setPower(extensionPower);


                controlRR.motorAngle1.setTargetPosition(currentAnglePosition);
                controlRR.motorAngle2.setTargetPosition(currentAnglePosition);

                controlRR.motorAngle1.setTargetPositionTolerance(angleTolerance);
                controlRR.motorAngle2.setTargetPositionTolerance(angleTolerance);

                controlRR.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                controlRR.motorAngle1.setPower(1);
                controlRR.motorAngle2.setPower(1);

                return false;
            }

        }
        public Result manualUp(){
            return new Result(new manualUp(), "manualUp");
        }

        public class manualRetract implements Action{
            boolean initialized = false;
            int additivePosition1;
            int additivePosition2;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized){
                    currentExtensionPosition -= extensionIncrement;
                    initialized = true;
                }

                controlRR.motorExtension1.setTargetPosition(currentExtensionPosition);
                controlRR.motorExtension2.setTargetPosition(currentExtensionPosition);

                controlRR.motorExtension1.setTargetPositionTolerance(extensionTolerance);
                controlRR.motorExtension2.setTargetPositionTolerance(extensionTolerance);

                controlRR.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorExtension1.setPower(1);
                controlRR.motorExtension2.setPower(1);

                return false;
            }
        }
        public Result manualRetract(){
            return new Result(new manualRetract(), "manualRetract");
        }

        public class manualExtend implements Action {
            boolean initialized = false;
            int additivePosition1;
            int additivePosition2;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    currentExtensionPosition += extensionIncrement;
                    initialized = true;
                }

                controlRR.motorExtension1.setTargetPosition(currentExtensionPosition);
                controlRR.motorExtension2.setTargetPosition(currentExtensionPosition);

                controlRR.motorExtension1.setTargetPositionTolerance(extensionTolerance);
                controlRR.motorExtension2.setTargetPositionTolerance(extensionTolerance);

                controlRR.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorExtension1.setPower(1);
                controlRR.motorExtension2.setPower(1);

                return false;
            }
        }

        public Result manualExtend(){
            return new Result(new manualExtend(), "manualExtend");
        }

        public class manualDown implements Action{
            boolean initialized = false;
            int additivePosition1;
            int additivePosition2;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized){
                    currentAnglePosition -= 30;
                    initialized = true;
                }

                currentExtensionPosition += (int) (angleIncrement * extendRatio);

                //set position of lift
                controlRR.motorExtension1.setTargetPosition(currentExtensionPosition);
                controlRR.motorExtension2.setTargetPosition(currentExtensionPosition);


                controlRR.motorExtension1.setTargetPositionTolerance(extensionTolerance);
                controlRR.motorExtension2.setTargetPositionTolerance(extensionTolerance);

                //set mode to run to position
                controlRR.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                controlRR.motorExtension1.setPower(extensionPower);
                controlRR.motorExtension2.setPower(extensionPower);


                controlRR.motorAngle1.setTargetPosition(currentAnglePosition);
                controlRR.motorAngle2.setTargetPosition(currentAnglePosition);

                controlRR.motorAngle1.setTargetPositionTolerance(angleTolerance);
                controlRR.motorAngle2.setTargetPositionTolerance(angleTolerance);

                controlRR.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorAngle1.setPower(1);
                controlRR.motorAngle2.setPower(1);

                return false;
            }
        }
        public Result manualDown(){
            return new Result(new manualDown(), "manualDown");
        }

        public class SetSwingPosition implements Action{
            boolean initialized = false;
            double position;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                }

                controlRR.swing.setPosition(position);

                return !(controlRR.swing.getPosition() == position);
            }

            public SetSwingPosition(double pos) {
                position = pos;
            }
        }
        public Result setSwingPosition(double pos){
            return new Result(new SetSwingPosition(pos), "setSwingPosition");
        }

        public class SetServoAnglePosition implements Action{
            boolean initialized = false;
            double position;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                }

                controlRR.angle.setPosition(position);

                return false;
            }

            public SetServoAnglePosition(double pos) {
                position = pos;
            }
        }
        public Result setServoAnglePosition(double pos){
            return new Result(new SetServoAnglePosition(pos), "setServoAnglePosition");
        }

        public class ClawOpen implements Action{
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                }

                controlRR.claw.setPosition(0.5);

                return false;
            }
        }
        public Result clawOpen(){
            return new Result(new ClawOpen(), "clawOpen");
        }

        public class ClawClose implements Action{
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                }

                controlRR.claw.setPosition(0.33);

                return false;
            }
        }
        public Result clawClose(){
            return new Result(new ClawClose(), "clawClose");
        }

        public class OpenCVPickup implements Action{
            boolean initialized = false;
            MecanumDrive drive;
            Pose2d pose;
            double anglePos;
            Action action;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                }
                anglePos = States.PICKUP.anglePosition + alignment.angle/300;

                controlRR.angle.setPosition(anglePos);

                return action.run(telemetryPacket) && !(controlRR.angle.getPosition() == anglePos);
            }

            public OpenCVPickup(MecanumDrive drive, Pose2d pose) {
                this.drive = drive;
                this.pose = pose;
                this.action = drive.actionBuilder(pose)
                        //change with conversion
                        .strafeTo(new Vector2d(pose.position.x + (alignment.deltaX * 72/144), pose.position.y + (alignment.deltaY * 72/144)))
                        .build();
            }
        }
        public Result openCVPickup(MecanumDrive drive, Pose2d pose){
            return new Result(new OpenCVPickup(drive, pose), "openCVPickup");
        }

        public class ZeroExtension implements Action{
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                }

                controlRR.motorAngle1.setTargetPosition(currentAnglePosition);
                controlRR.motorAngle2.setTargetPosition(currentAnglePosition);
                controlRR.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                controlRR.motorAngle1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                controlRR.motorAngle2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                while (!controlRR.limitSwitchExtension.isPressed()) {
                    Actions.runBlocking(manualRetract().getAction());
                }

                currentExtensionPosition = 50;
                controlRR.motorExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                controlRR.motorExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                return false;
            }
        }
        public Result zeroExtension(){
            return new Result(new ZeroExtension(), "zeroExtension");
        }

        public class ZeroAngle implements Action{
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                }

                while (!controlRR.limitSwitchAngle.isPressed()) {
                    Actions.runBlocking(manualDown().getAction());
                }

                currentAnglePosition = 10;
                controlRR.motorAngle1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                controlRR.motorAngle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                return false;
            }
        }
        public Result zeroAngle(){
            return new Result(new ZeroAngle(), "zeroAngle");
        }

        public class ManualClawAngle implements Action{
            boolean initialized = false;
            double stepValue;
            double currentPosition;
            double targetPosition;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    currentPosition = controlRR.angle.getPosition();
                    targetPosition = currentPosition+stepValue;
                }

                if (targetPosition > 1) {
                    targetPosition = 1;
                } else if (targetPosition < 0) {
                    targetPosition = 0;
                }
                controlRR.angle.setPosition(targetPosition);

                return false;
            }

            public ManualClawAngle(double step) {
                stepValue = step;
            }
        }
        public Result manualClawAngle(double step) {
            return new Result(new ManualClawAngle(step), "manualClawAngle");
        }

        public Result lightOff() {
//            return new Result(new SequentialAction(
//                    new InstantAction(() -> controlRR.light(false)),
//                    new InstantAction(() -> controlRR.light.setMode(DigitalChannel.Mode.OUTPUT))
//            ), "lightOff");
            return new Result(new InstantAction(() ->  telemetry.addLine("Live Laugh Lockheed.")), "lightOff");
        }

        public Result lightOn() {
//            return new Result(new SequentialAction(
//                    new InstantAction(() -> controlRR.light.setState(true)),
//                    new InstantAction(() -> controlRR.light.setMode(DigitalChannel.Mode.OUTPUT))
//            ), "lightOn");
            return new Result(new InstantAction(() ->  telemetry.addLine("Get the bag with Grumman.")), "lightOn");
        }

        public Result drop() {
            return new Result(new SequentialAction(
                    setExtensionPosition(0).getAction(),
                    setAnglePosition(States.DROP.motorAnglePosition).getAction(),
                    setExtensionPosition(States.DROP.motorExtensionPosition).getAction(),
                    setSwingPosition(States.DROP.swingPosition).getAction()
            ), "drop");
        }

        public class DriveControl implements Action {
            boolean initialized = false;
            private MecanumDrive drive;
            private Action action;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                }

                if (action.run(telemetryPacket)) {
                    return false;
                }
                else {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                            -gamepad1.right_stick_x));
                    return true;
                }
            }

            public DriveControl(MecanumDrive d, Action action) {
                this.drive = d;
                this.action = action;
            }
        }
        public Result driveControl(MecanumDrive drive, Action action){
            return new Result(new DriveControl(drive, action), "driveControl");
        }

        public Result wallPickup() {
            return new Result(new SequentialAction(
                    setServoAnglePosition(States.WALLPICKUP.anglePosition).getAction(),
                    setSwingPosition(0).getAction(),
                    setExtensionPosition(100).getAction(),
                    setAnglePosition(States.WALLPICKUP.motorAnglePosition).getAction(),
                    new ParallelAction(
                            setExtensionPosition(States.WALLPICKUP.motorExtensionPosition).getAction(),
                            setSwingPosition(States.WALLPICKUP.swingPosition).getAction()
                    )
            ), "wallPickup");
        }

        public Result pickup() {
            return new Result(new SequentialAction(
                    new ParallelAction(
                            lightOn().getAction(),
                            setSwingPosition(0).getAction(),
                            setExtensionPosition(100).getAction(),
                            setServoAnglePosition(States.PICKUP.anglePosition).getAction()
                    ),
                    setAnglePosition(States.PICKUP.motorAnglePosition).getAction(),
                    setExtensionPosition(States.PICKUP.motorExtensionPosition).getAction(),
                    setSwingPosition(States.PICKUP.swingPosition).getAction()
            ), "pickup");
        }

        public Result prepareClip() {
            return new Result(new SequentialAction(
                    new ParallelAction(
                            setSwingPosition(States.PREPARECLIP.swingPosition).getAction(),
                            setServoAnglePosition(States.PREPARECLIP.anglePosition).getAction(),
                            setExtensionPosition(100).getAction()
                    ),
                    new ParallelAction(
                            setAnglePosition(States.PREPARECLIP.motorAnglePosition).getAction(),
                            setExtensionPosition(States.PREPARECLIP.motorExtensionPosition).getAction()
                    )
            ), "prepareClip");
        }

        public Result clipClip() {
            return new Result(new SequentialAction(
                    setAnglePosition(States.CLIPCLIP.motorAnglePosition).getAction(),
                    setExtensionPosition(States.CLIPCLIP.motorExtensionPosition).getAction(),
                    new ParallelAction(
                            setAnglePosition(States.CLIPCLIP.motorAnglePosition-70).getAction(),
                            clawOpen().getAction(),
                            setServoAnglePosition(States.CLIPCLIP.anglePosition).getAction()
                    ),
                    setExtensionPosition(States.PREPARECLIP.motorExtensionPosition).getAction()
                    //pickup().getAction()
                    //rest().getAction()
            ), "clipClip");
        }

        public Result rest() {
            return new Result(new SequentialAction(
                    lightOff().getAction(),
                    new ParallelAction(
                            setSwingPosition(1.0).getAction(),
                            setExtensionPosition(20).getAction()
                    ),
                    setAnglePosition(100).getAction(),
                    setSwingPosition(0.5).getAction()
            ), "rest");
        }

        public Result pickupSample(MecanumDrive drive, Pose2d pose) {
            return new Result(new SequentialAction(
//                    ope(drive, pose).getAction(),
//                    new SleepAction(1),
                    setAnglePosition(83).getAction(),
                    clawClose().getAction(),
                    setAnglePosition(States.PICKUP.motorAnglePosition).getAction()
                    //new SleepAction(0.1)
                    //lightOff().getAction(),
                    //rest().getAction()
            ), "pickupSample");
        }

        public Result hangPrepare() {
            return new Result(new SequentialAction(
                    new ParallelAction(
                            setExtensionPosition(50).getAction(),
                            setSwingPosition(States.HANGPREPARE.swingPosition).getAction()
                    ),
                    setAnglePosition(States.HANGPREPARE.motorAnglePosition).getAction(),
                    setExtensionPosition(States.HANGPREPARE.motorExtensionPosition).getAction()
            ), "hangPrepare");
        }

        public Result hang() {
            return new Result(new SequentialAction(
                    setExtensionPosition(States.HANG.motorExtensionPosition-(States.HANG.motorExtensionPosition/2)).getAction(),
                    new ParallelAction(
                            setAnglePosition(States.HANG.motorAnglePosition).getAction(),
                            setExtensionPosition(States.HANG.motorExtensionPosition).getAction()
                    )
            ), "hang");
        }

    }
    FtcDashboard dash = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public abstract void runOpMode() throws InterruptedException;
    public class RedProcessor implements VisionProcessor {
        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            // Not useful in this case, but we do need to implement it either way
        }

        public class Alignment {
            public double deltaX;
            public double deltaY;
            public double angle;
            public Alignment(double x, double y, double a) {
                this.deltaX = x;
                this.deltaY = y;
                this.angle = a;
            }
        }

        private Mat output;
        private Mat mask;
        private Mat mask1;
        private Mat mask2;
        private Mat hierarchy;
        private List<MatOfPoint> contours;
        private List<RotatedRect> rects;
        private MatOfPoint2f contour2f;
        private MatOfPoint2f approx;
        public String COLOR = "RED";

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            if (frame == null) return null;

            double BLOCK_HEIGHT = 100;
            double BLOCK_WIDTH = 50;
            double INCHES_PER_PIXEL = 0.014625;

            // Set a point at 1/3 from the bottom of the image and 1/2 from the left.
            Point target_point = new Point(frame.cols() / 2.0, frame.rows() * 2 / 3.0);


            // Convert the frame to HSV
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
            output = new Mat();

            int erode_int = 5;
            int dilate_int = 9;

            if (COLOR.equals("RED")) {
                Scalar lower1 = new Scalar(0, 0, 30);
                Scalar upper1 = new Scalar(10, 255, 255);
                Scalar lower2 = new Scalar(170, 20, 100);
                Scalar upper2 = new Scalar(179, 255, 255);

                mask1 = new Mat();
                mask2 = new Mat();
                Core.inRange(frame, lower1, upper1, mask1);
                Core.inRange(frame, lower2, upper2, mask2);

                mask = new Mat();
                Core.add(mask1, mask2, mask);

                frame.copyTo(output, mask);

            } else if (COLOR.equals("BLUE")) {
                Scalar lower = new Scalar(100, 50, 0);
                Scalar upper = new Scalar(140, 255, 255);
                erode_int = 9;
                dilate_int = 13;
                mask = new Mat();
                Core.inRange(frame, lower, upper, mask);

                frame.copyTo(output, mask);
            } else if (COLOR.equals("YELLOW")) {
                Scalar lower = new Scalar(10, 90, 50);
                Scalar upper = new Scalar(50, 255, 255);

                mask = new Mat();
                Core.inRange(frame, lower, upper, mask);

                frame.copyTo(output, mask);
            }

            // Ignore the bottom 1/3 of the image.
//        Rect roi = new Rect(0, 0, frame.cols(), frame.rows() * 2 / 3);
//        output = new Mat(output, roi);

            Imgproc.erode(output, output, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(erode_int, erode_int)));
            Imgproc.dilate(output, output, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(dilate_int, dilate_int)));

            // Convert the output to grayscale
            Imgproc.cvtColor(output, output, Imgproc.COLOR_BGR2GRAY);

            // Find contours
            contours = new ArrayList<>();
            rects = new ArrayList<>();
            hierarchy = new Mat();
            Imgproc.findContours(output, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            Point imgCenter = new Point(frame.cols() / 2.0, frame.rows() / 2.0);
            double minDist = Double.MAX_VALUE;
            RotatedRect minRect = null;

            // Find the closest rectangle to the center of the image.
            for (MatOfPoint contour : contours) {
                contour2f = new MatOfPoint2f(contour.toArray());
                approx = new MatOfPoint2f();
                Imgproc.approxPolyDP(contour2f, approx, 0.02 * Imgproc.arcLength(contour2f, true), true);

                // We only care about rectangles with 4 corners that are close to the expected size.
                if (approx.toList().size() >= 4) {
                    RotatedRect rect = Imgproc.minAreaRect(approx);
                    // Check if the rectangle is bigger than the block size.
                    if (rect.size.height < BLOCK_HEIGHT || rect.size.width < BLOCK_WIDTH) {
                        continue;
                    }
                    Point[] points = new Point[4];
                    rect.points(points);

                    double dist = 0;
                    for (Point p : points) {
                        dist += Math.sqrt(Math.pow(p.x - target_point.x, 2) + Math.pow(p.y - target_point.y, 2));
                    }

                    if (dist < minDist) {
                        minDist = dist;
                        minRect = rect;
                    }

                    // rects.add(rect);
                }
            }

            double delta_x = 0;
            double delta_y = 0;
            double angle = 0;

            frame.release();

            if (minRect != null) {
                delta_x = target_point.x - minRect.center.x;
                delta_y = target_point.y - minRect.center.y;
                angle = minRect.angle;
                // If the width is greater than the height, the angle is off by 90 degrees.
                if (minRect.size.width > minRect.size.height) {
                    angle += 90;
                }
                // Make sure the angle is between -180 and 180 normalized to be from a vertical line.
                if (angle > 90) {
                    angle -= 180;
                } else if (angle < -90) {
                    angle += 180;
                }

                // Convert pixels to inches to move.
                delta_x *= INCHES_PER_PIXEL;
                delta_y *= INCHES_PER_PIXEL;
            }

            alignment = new Alignment(delta_x, delta_y, angle);
            return alignment;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        }
    }

}
