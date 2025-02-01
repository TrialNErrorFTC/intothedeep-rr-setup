package org.firstinspires.ftc.teamcode;

import android.net.wifi.p2p.WifiP2pManager;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nonRR.States;
import org.firstinspires.ftc.teamcode.robothardware.ControlRR;

public abstract class TeleOpActionsRR extends LinearOpMode {
    ControlRR controlRR;
    private double extensionPower= 0.6;
    private double anglePower = 0.5;
    private double extensionHoldPower = 0.4;
    private double angleHoldPower = 0.4;

    private int extensionTolerance = 3;
    private int angleTolerance = 8;

    // Store current motor value targets
    private int currentExtensionPosition = 0;
    private int currentAnglePosition = 0;

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
            t.addData("motorExtension2", controlRR.motorExtension2.getCurrentPosition());
            t.addData("motorAngle1", controlRR.motorAngle1.getCurrentPosition());
            t.addData("motorAngle2", controlRR.motorAngle2.getCurrentPosition());
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

                controlRR.motorAngle1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                controlRR.motorAngle2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                currentAnglePosition = anglePosition;

                //set position of lift
                controlRR.motorAngle1.setTargetPosition(anglePosition);
                controlRR.motorAngle2.setTargetPosition(anglePosition);

                //set mode to run to position
                controlRR.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                controlRR.motorAngle1.setPower(anglePower);
                controlRR.motorAngle2.setPower(anglePower);

                if (Math.abs(controlRR.motorAngle1.getCurrentPosition() - anglePosition) < angleTolerance*2 || (controlRR.motorAngle1.getCurrentPosition() - anglePosition) < angleTolerance*2){
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

                controlRR.motorExtension1.setPower(extensionPower);
                controlRR.motorExtension2.setPower(extensionPower);

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

                controlRR.motorAngle1.setPower(anglePower);
                controlRR.motorAngle2.setPower(anglePower);

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
                    currentAnglePosition += 30;
                    initialized = true;
                }

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
                    currentExtensionPosition -= 50;
                    initialized = true;
                }

                controlRR.motorExtension1.setTargetPosition(currentExtensionPosition);
                controlRR.motorExtension2.setTargetPosition(currentExtensionPosition);

                controlRR.motorExtension1.setTargetPositionTolerance(10);
                controlRR.motorExtension2.setTargetPositionTolerance(10);

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
                    currentExtensionPosition += 50;
                    initialized = true;
                }

                controlRR.motorExtension1.setTargetPosition(currentExtensionPosition);
                controlRR.motorExtension2.setTargetPosition(currentExtensionPosition);

                controlRR.motorExtension1.setTargetPositionTolerance(10);
                controlRR.motorExtension2.setTargetPositionTolerance(10);

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

                controlRR.motorAngle1.setTargetPosition(currentAnglePosition);
                controlRR.motorAngle2.setTargetPosition(currentAnglePosition);

                controlRR.motorAngle1.setTargetPositionTolerance(10);
                controlRR.motorAngle2.setTargetPositionTolerance(10);

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

                return false;
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

                controlRR.claw.setPosition(0.35);

                return false;
            }
        }
        public Result clawClose(){
            return new Result(new ClawClose(), "clawClose");
        }

        public class OpenCVPickup implements Action{
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                }

                return false;
            }
        }
        public Result openCVPickup(){
            return new Result(new OpenCVPickup(), "openCVPickup");
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
            return new Result(new SequentialAction(
                    new InstantAction(() -> controlRR.light.setState(false)),
                    new InstantAction(() -> controlRR.light.setMode(DigitalChannel.Mode.OUTPUT))
            ), "lightOff");
        }

        public Result lightOn() {
            return new Result(new SequentialAction(
                    new InstantAction(() -> controlRR.light.setState(true)),
                    new InstantAction(() -> controlRR.light.setMode(DigitalChannel.Mode.OUTPUT))
            ), "lightOn");
        }

        public Result drop() {
            return new Result(new SequentialAction(
                    setExtensionPosition(0).getAction(),
                    setAnglePosition(States.DROP.motorAnglePosition).getAction(),
                    setExtensionPosition(States.DROP.motorExtensionPosition).getAction(),
                    setSwingPosition(States.DROP.swingPosition).getAction()
            ), "drop");
        }

        public class DriveControl implements Action{
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
                    setAnglePosition(States.PICKUP.motorAnglePosition).getAction(),
                    setSwingPosition(States.PICKUP.swingPosition).getAction()
            ), "pickup");
        }

        public Result prepareClip() {
            return new Result(new SequentialAction(
                    new ParallelAction(
                            setSwingPosition(States.PREPARECLIP.swingPosition).getAction(),
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
                            setAnglePosition(States.CLIPCLIP.motorAnglePosition-75).getAction(),
                            clawOpen().getAction(),
                            setServoAnglePosition(0.5).getAction()
                    ),
                    setExtensionPosition(States.PREPARECLIP.motorExtensionPosition).getAction(),
                    rest().getAction()
            ), "clipClip");
        }

        public Result rest() {
            return new Result(new SequentialAction(
                    new ParallelAction(
                            setSwingPosition(1.0).getAction(),
                            setExtensionPosition(50).getAction()
                    ),
                    setAnglePosition(50).getAction(),
                    setSwingPosition(0.5).getAction()
            ), "rest");
        }

        public Result pickupSample() {
            return new Result(new SequentialAction(
                    setAnglePosition(10).getAction(),
                    new SleepAction(0.45),
                    clawClose().getAction(),
                    setAnglePosition(States.PICKUP.motorAnglePosition).getAction(),
                    new SleepAction(0.45),
                    lightOff().getAction(),
                    rest().getAction()
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
}