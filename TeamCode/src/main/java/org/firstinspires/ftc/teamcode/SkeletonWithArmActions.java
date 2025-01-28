package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robothardware.Lift;

import java.util.ArrayList;
import java.util.List;

public abstract class SkeletonWithArmActions extends LinearOpMode {
    int extensionPosition = 100;
    int anglePosition = 0;
    Lift lift;

    public class LiftWithActions {

        public LiftWithActions(){
            lift = new Lift(hardwareMap);
        }

        public class setExtensionPosition implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                    //set position of lift
                    lift.motorExtension1.setTargetPosition(extensionPosition);
                    lift.motorExtension2.setTargetPosition(extensionPosition);

                    //set mode to run to position
                    lift.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    lift.motorExtension1.setPower(0.25);
                    lift.motorExtension2.setPower(0.25);

                    if (Math.abs(lift.motorExtension1.getCurrentPosition() - extensionPosition) < 10){
                        return false;
                    } else {
                        return true;
                    }
            }
        }
        public Action setExtensionPosition() {
            return new setExtensionPosition();
        }



        public class retainPosition implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized){
                }

                initialized = true;
                return false;

            }
        }
        public Action retainPosition() {
            return new retainPosition();
        }

        public class manualExtend implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized){
                }

                initialized = true;
                return false;

            }
        }
        public Action manualExtend() {
            return new manualExtend();
        }
        public class manualRetract implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized){

                }

                initialized = true;
                return false;

            }
        }
        public Action manualRetract() {
            return new manualRetract();
        }
        public class manualUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized){
                }

                initialized = true;
                return false;

            }
        }
        public Action manualUp() {
            return new manualUp();
        }
        public class manualDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized){

                }

                initialized = true;
                return false;

            }
        }
        public Action manualDown() {
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
