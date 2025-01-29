package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.nonRR.States;
import org.firstinspires.ftc.teamcode.robothardware.Lift;


import java.util.ArrayList;
import java.util.List;

public abstract class SkeletonWithArmActions extends LinearOpMode {
    Lift lift;
    private double extensionPower= 0.25;
    private double anglePower = 0.25;
    private double extensionHoldPower = 0.4;
    private double angleHoldPower = 0.4;

    private int extensionTolerance = 10;
    private int angleTolerance = 10;

    public class LiftWithActions {
        public LiftWithActions(){
            lift = new Lift(hardwareMap);
        }

        public class setExtensionPosition implements Action {
            private final int extensionPosition;
            private boolean initialized = false;

            public setExtensionPosition(int extensionPosition) {
                this.extensionPosition = extensionPosition;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                    //set position of lift
                    lift.motorExtension1.setTargetPosition(extensionPosition);
                    lift.motorExtension2.setTargetPosition(extensionPosition);

                    //set mode to run to position
                    lift.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //lift.motorExtension1.setPower(0.25);
                    //lift.motorExtension2.setPower(0.25);

                    lift.motorExtension1.setPower(extensionPower);
                    lift.motorExtension2.setPower(extensionPower);

                    if (Math.abs(lift.motorExtension1.getCurrentPosition() - extensionPosition) < 10){
                        return false;
                    } else {
                        return true;
                    }
            };
        }
        public Action setExtensionPosition(int position) {
            return new setExtensionPosition(position);
        }

        public class setAnglePosition implements Action {
            private boolean initialized = false;
            private int anglePosition;

            public setAnglePosition(int anglePosition) {
                this.anglePosition = anglePosition;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //set position of lift
                lift.motorAngle1.setTargetPosition(anglePosition);
                lift.motorAngle2.setTargetPosition(anglePosition);

                //set mode to run to position
                lift.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//                lift.motorAngle1.setPower(0.25);
//                lift.motorAngle2.setPower(0.25);

                lift.motorAngle1.setPower(anglePower);
                lift.motorAngle2.setPower(anglePower);

                if (Math.abs(lift.motorAngle1.getCurrentPosition() - anglePosition) < 10 || (lift.motorAngle1.getCurrentPosition() - anglePosition) < 10){
                    return false;
                } else {
                    return true;
                }
            };
        }
        public Action setAnglePosition(int position) {
            return new setAnglePosition(position);
        }


        public class retainExtensionPosition implements Action {
            private boolean initialized = false;
            private int currentPosition1;
            private int currentPosition2;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized){
                    currentPosition1 = lift.motorExtension1.getCurrentPosition();
                    currentPosition2 = lift.motorExtension2.getCurrentPosition();
                    initialized = true;
                }

                // keep the motor in a position that is in the tolerance range, and ends when going to the next action
                lift.motorExtension1.setTargetPosition(currentPosition1);
                lift.motorExtension2.setTargetPosition(currentPosition2);
                lift.motorExtension1.setTargetPositionTolerance(extensionTolerance);
                lift.motorExtension2.setTargetPositionTolerance(extensionTolerance);
                lift.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//                lift.motorExtension1.setPower(0.25);
//                lift.motorExtension2.setPower(0.25);

                lift.motorExtension1.setPower(extensionPower);
                lift.motorExtension2.setPower(extensionPower);

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
                lift.motorAngle1.setTargetPosition(lift.motorAngle1.getCurrentPosition());
                lift.motorAngle2.setTargetPosition(lift.motorAngle2.getCurrentPosition());

                lift.motorAngle1.setTargetPositionTolerance(angleTolerance);
                lift.motorAngle2.setTargetPositionTolerance(angleTolerance);

                lift.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//                lift.motorAngle1.setPower(0.25);
//                lift.motorAngle2.setPower(0.25);

                lift.motorAngle1.setPower(anglePower);
                lift.motorAngle2.setPower(anglePower);

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
                    additivePosition1 = lift.motorAngle1.getCurrentPosition() + 30;
                    additivePosition2 = lift.motorAngle2.getCurrentPosition() + 30;
                    initialized = true;
                }

                lift.motorAngle1.setTargetPosition(additivePosition1);
                lift.motorAngle2.setTargetPosition(additivePosition2);

                lift.motorAngle1.setTargetPositionTolerance(angleTolerance);
                lift.motorAngle2.setTargetPositionTolerance(angleTolerance);

                lift.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//                lift.motorAngle1.setPower(0.25);
//                lift.motorAngle2.setPower(0.25);

                lift.motorAngle1.setPower(anglePower);
                lift.motorAngle2.setPower(anglePower);

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
                        additivePosition1 = lift.motorExtension1.getCurrentPosition() - 30;
                        additivePosition2 = lift.motorExtension2.getCurrentPosition() - 30;
                        initialized = true;
                    }

                    lift.motorExtension1.setTargetPosition(additivePosition1);
                    lift.motorExtension2.setTargetPosition(additivePosition2);

                    lift.motorExtension1.setTargetPositionTolerance(10);
                    lift.motorExtension2.setTargetPositionTolerance(10);

                    lift.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.motorExtension1.setPower(0.25);
                    lift.motorExtension2.setPower(0.25);


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
                    additivePosition1 = lift.motorExtension1.getCurrentPosition() + 30;
                    additivePosition2 = lift.motorExtension2.getCurrentPosition() + 30;
                    initialized = true;
                }

                lift.motorExtension1.setTargetPosition(additivePosition1);
                lift.motorExtension2.setTargetPosition(additivePosition2);

                lift.motorExtension1.setTargetPositionTolerance(10);
                lift.motorExtension2.setTargetPositionTolerance(10);

                lift.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.motorExtension1.setPower(0.25);
                lift.motorExtension2.setPower(0.25);


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
                    additivePosition1 = lift.motorAngle1.getCurrentPosition() - 30;
                    additivePosition2 = lift.motorAngle2.getCurrentPosition() - 30;
                    initialized = true;
                }

                lift.motorAngle1.setTargetPosition(additivePosition1);
                lift.motorAngle2.setTargetPosition(additivePosition2);

                lift.motorAngle1.setTargetPositionTolerance(10);
                lift.motorAngle2.setTargetPositionTolerance(10);

                lift.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.motorAngle1.setPower(0.25);
                lift.motorAngle2.setPower(0.25);


                return false;
            }
        }
        public Action manualDown(){
            return new manualDown();
        }



    }
    public class Claw {

    }
    FtcDashboard dash = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public abstract void runOpMode() throws InterruptedException;
}
