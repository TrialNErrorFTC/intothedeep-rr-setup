package org.firstinspires.ftc.teamcode;

import android.net.wifi.p2p.WifiP2pManager;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.nonRR.States;
import org.firstinspires.ftc.teamcode.robothardware.ControlRR;

public abstract class TeleOpActionsRR extends LinearOpMode {
    ControlRR controlRR;
    private double extensionPower= 0.6;
    private double anglePower = 0.4;
    private double extensionHoldPower = 0.4;
    private double angleHoldPower = 0.4;

    private int extensionTolerance = 10;
    private int angleTolerance = 10;

    // Store current motor value targets
    private int currentExtensionPosition = 0;
    private int currentAnglePosition = 0;

    public class ActionControl {
        public ActionControl(){
            controlRR = new ControlRR(hardwareMap);
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

                    //lift.motorExtension1.setPower(0.25);
                    //lift.motorExtension2.setPower(0.25);

                    controlRR.motorExtension1.setPower(extensionPower);
                    controlRR.motorExtension2.setPower(extensionPower);

                    if (Math.abs(controlRR.motorExtension1.getCurrentPosition() - extensionPosition) < 10){
                        return false;
                    } else {
                        return true;
                    }
            };
        }
        public Action setExtensionPosition(int position) {
            return new setExtensionPosition(position);
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

//                lift.motorAngle1.setPower(0.25);
//                lift.motorAngle2.setPower(0.25);

                controlRR.motorAngle1.setPower(anglePower);
                controlRR.motorAngle2.setPower(anglePower);

                if (Math.abs(controlRR.motorAngle1.getCurrentPosition() - anglePosition) < 10 || (controlRR.motorAngle1.getCurrentPosition() - anglePosition) < 10){
                    return false;
                } else {
                    return true;
                }
            };
        }
        public Action setAnglePosition(int position) {
            return new SetAnglePosition(position);
        }


        public class retainExtensionPosition implements Action {
            private boolean initialized = false;
            private int currentPosition1;
            private int currentPosition2;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized){
                    //currentPosition1 = controlRR.motorExtension1.getCurrentPosition();
                    //currentPosition2 = controlRR.motorExtension2.getCurrentPosition();
                    initialized = true;
                }

                // keep the motor in a position that is in the tolerance range, and ends when going to the next action
                controlRR.motorExtension1.setTargetPosition(currentExtensionPosition);
                controlRR.motorExtension2.setTargetPosition(currentExtensionPosition);
                controlRR.motorExtension1.setTargetPositionTolerance(extensionTolerance);
                controlRR.motorExtension2.setTargetPositionTolerance(extensionTolerance);
                controlRR.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//                lift.motorExtension1.setPower(0.25);
//                lift.motorExtension2.setPower(0.25);

                controlRR.motorExtension1.setPower(extensionPower);
                controlRR.motorExtension2.setPower(extensionPower);

                return false;
            }
        }
        public Action retainExtensionPosition() {
            return new retainExtensionPosition();
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


//                lift.motorAngle1.setPower(0.25);
//                lift.motorAngle2.setPower(0.25);

                controlRR.motorAngle1.setPower(anglePower);
                controlRR.motorAngle2.setPower(anglePower);

                return false;
            }
        }
        public Action retainAnglePosition() {
            return new retainAnglePosition();
        }

        public class manualUp implements Action {
            boolean initialized = false;
            int additivePosition1;
            int additivePosition2;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                if (!initialized) {

                    currentAnglePosition += 5;
                    //additivePosition1 = lift.motorAngle1.getCurrentPosition() + 30;
                    //additivePosition2 = lift.motorAngle2.getCurrentPosition() + 30;
                    initialized = true;
                }

                controlRR.motorAngle1.setTargetPosition(currentAnglePosition);
                controlRR.motorAngle2.setTargetPosition(currentAnglePosition);

                controlRR.motorAngle1.setTargetPositionTolerance(angleTolerance);
                controlRR.motorAngle2.setTargetPositionTolerance(angleTolerance);

                controlRR.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//                lift.motorAngle1.setPower(0.25);
//                lift.motorAngle2.setPower(0.25);

                controlRR.motorAngle1.setPower(anglePower);
                controlRR.motorAngle2.setPower(anglePower);

                return false;
            }

        }
        public Action manualUp(){
            return new manualUp();
        }
        public class manualRetract implements Action{
                boolean initialized = false;
                int additivePosition1;
                int additivePosition2;

                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                    if (!initialized){
                        currentExtensionPosition -= 30;
                        //additivePosition1 = controlRR.motorExtension1.getCurrentPosition() - 30;
                        //additivePosition2 = controlRR.motorExtension2.getCurrentPosition() - 30;
                        initialized = true;
                    }

                    controlRR.motorExtension1.setTargetPosition(currentExtensionPosition);
                    controlRR.motorExtension2.setTargetPosition(currentExtensionPosition);

                    controlRR.motorExtension1.setTargetPositionTolerance(10);
                    controlRR.motorExtension2.setTargetPositionTolerance(10);

                    controlRR.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    controlRR.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    controlRR.motorExtension1.setPower(extensionPower);
                    controlRR.motorExtension2.setPower(extensionPower);


                    return false;
                }
        }
        public Action manualRetract(){
            return new manualRetract();
        }
        public class manualExtend implements Action {
            boolean initialized = false;
            int additivePosition1;
            int additivePosition2;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                if (!initialized) {
                    currentExtensionPosition += 30;
                    //additivePosition1 = controlRR.motorExtension1.getCurrentPosition() + 30;
                    //additivePosition2 = controlRR.motorExtension2.getCurrentPosition() + 30;
                    initialized = true;
                }

                controlRR.motorExtension1.setTargetPosition(currentExtensionPosition);
                controlRR.motorExtension2.setTargetPosition(currentExtensionPosition);

                controlRR.motorExtension1.setTargetPositionTolerance(10);
                controlRR.motorExtension2.setTargetPositionTolerance(10);

                controlRR.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorExtension1.setPower(extensionPower);
                controlRR.motorExtension2.setPower(extensionPower);


                return false;
            }

        }
        public Action manualExtend(){
            return new manualExtend();
        }
        public class manualDown implements Action{
            boolean initialized = false;
            int additivePosition1;
            int additivePosition2;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                if (!initialized){
                    currentAnglePosition -= 30;
                    //additivePosition1 = controlRR.motorAngle1.getCurrentPosition() - 30;
                    //additivePosition2 = controlRR.motorAngle2.getCurrentPosition() - 30;
                    initialized = true;
                }

                controlRR.motorAngle1.setTargetPosition(currentAnglePosition);
                controlRR.motorAngle2.setTargetPosition(currentAnglePosition);

                controlRR.motorAngle1.setTargetPositionTolerance(10);
                controlRR.motorAngle2.setTargetPositionTolerance(10);

                controlRR.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                controlRR.motorAngle1.setPower(anglePower);
                controlRR.motorAngle2.setPower(anglePower);


                return false;
            }
        }
        public Action manualDown(){
            return new manualDown();
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
        public Action setSwingPosition(double pos){
            return new SetSwingPosition(pos);
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

                return !(controlRR.angle.getPosition() == position);
            }

            public SetServoAnglePosition(double pos) {
                position = pos;
            }
        }
        public Action setServoAnglePosition(double pos){
            return new SetServoAnglePosition(pos);
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
        public Action clawOpen(){
            return new ClawOpen();
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
        public Action clawClose(){
            return new ClawClose();
        }

        /*
        // TODO
        // ADD OPEN CV TO MOVE ROBOT
        // USE TRAJECTORY BUILDER
         */

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
        public Action openCVPickup(){
            return new OpenCVPickup();
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
                    Actions.runBlocking(manualRetract());
                }

                currentExtensionPosition = 50;
                controlRR.motorExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                controlRR.motorExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                return false;
            }
        }
        public Action zeroExtension(){
            return new ZeroExtension();
        }


        public class ZeroAngle implements Action{
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                }

                while (!controlRR.limitSwitchAngle.isPressed()) {
                    Actions.runBlocking(manualDown());
                }

                currentAnglePosition = 10;
                controlRR.motorAngle1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                controlRR.motorAngle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                return false;
            }
        }
        public Action zeroAngle(){
            return new ZeroAngle();
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

                return !(controlRR.angle.getPosition() == targetPosition);
            }

            public ManualClawAngle(double step) {
                stepValue = step;
            }
        }
        public Action manualClawAngle(double step) {
            return new ManualClawAngle(step);
        }

        public Action drop() {
            return new SequentialAction(
                    setExtensionPosition(0),
                    setAnglePosition(States.DROP.motorAnglePosition),
                    setExtensionPosition(States.DROP.motorExtensionPosition)
            );
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
        public Action driveControl(MecanumDrive drive, Action action){
            return new DriveControl(drive, action);
        }


        public Action wallPickup() {
            return new SequentialAction(
                    setServoAnglePosition(States.WALLPICKUP.anglePosition),
                    setSwingPosition(0),
                    setExtensionPosition(100),
                    setAnglePosition(States.WALLPICKUP.motorAnglePosition),
                    new ParallelAction(
                            setExtensionPosition(States.WALLPICKUP.motorExtensionPosition),
                            setSwingPosition(States.WALLPICKUP.swingPosition)
                    )
            );
        }

        public Action pickup() {
            return new SequentialAction(
                    new ParallelAction(
                            setSwingPosition(0),
                            setExtensionPosition(100)
                    ),
                    setAnglePosition(States.PICKUP.motorAnglePosition),
                    setExtensionPosition(States.PICKUP.motorExtensionPosition),
                    setAnglePosition(States.PICKUP.motorAnglePosition),
                    setSwingPosition(States.PICKUP.swingPosition)
            );
        }

        public Action prepareClip() {
            return new SequentialAction(
                    new ParallelAction(
                        setSwingPosition(States.PREPARECLIP.swingPosition),
                        setExtensionPosition(100)
                    ),
                    new ParallelAction(
                            setAnglePosition(States.PREPARECLIP.motorAnglePosition),
                            setExtensionPosition(States.PREPARECLIP.motorExtensionPosition)
                    )
            );
        }

        public Action clipClip() {
            return new SequentialAction(
                    setAnglePosition(States.CLIPCLIP.motorAnglePosition),
                    setExtensionPosition(States.CLIPCLIP.motorExtensionPosition),
                    new ParallelAction(
                            setAnglePosition(States.CLIPCLIP.motorAnglePosition-75),
                            clawOpen(),
                            setServoAnglePosition(0.5)
                            ),
                    setExtensionPosition(States.PREPARECLIP.motorExtensionPosition),
                    rest()
            );
        }

        public Action rest() {
            return new SequentialAction(
              new ParallelAction(
                      setSwingPosition(100),
                      setExtensionPosition(50)
              ),
                    setAnglePosition(50),
                    setSwingPosition(0.5)
            );
        }

        public Action pickupSample() {
            return new SequentialAction(
                    //openCVPickup(), // TODO : Change this if needed for OpenCV
                    setAnglePosition(States.PICKUP.motorAnglePosition-150),
                    clawClose(),
                    setAnglePosition(States.PICKUP.motorAnglePosition)
                    //rest()
            );
        }

        public Action hangPrepare() {
            return new SequentialAction(
                    new ParallelAction(
                            setExtensionPosition(50),
                            setSwingPosition(States.HANGPREPARE.swingPosition)
                    ),
                    setAnglePosition(States.HANG.motorAnglePosition)
            );
        }

        public Action hang() {
            return new SequentialAction(
                    setExtensionPosition(States.HANG.motorExtensionPosition+(States.HANG.motorExtensionPosition/2)),
                    new ParallelAction(
                            setAnglePosition(States.HANG.motorAnglePosition),
                            setExtensionPosition(States.HANG.motorExtensionPosition)
                    )
            );
        }

    }
    FtcDashboard dash = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public abstract void runOpMode() throws InterruptedException;
}
