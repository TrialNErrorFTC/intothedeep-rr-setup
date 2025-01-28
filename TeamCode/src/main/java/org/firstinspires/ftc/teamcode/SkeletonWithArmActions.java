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
    int extensionPosition = 0;
    int anglePosition = 0;
    Lift lift;

    public class LiftWithActions {

        public LiftWithActions(){
            lift = new Lift(hardwareMap);
        }

        public class setLiftPosition implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized){
                    //set position of lift
                    lift.motorAngle1.setTargetPosition(anglePosition);
                    lift.motorAngle2.setTargetPosition(anglePosition);
                    lift.motorExtension1.setTargetPosition(extensionPosition);
                    lift.motorExtension2.setTargetPosition(extensionPosition);

                    //set mode to run to position
                    lift.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    //finish


                }

                initialized = true;
                return false;

            }
        }
        public Action setLiftPosition() {
            return new setLiftPosition();
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
            return new setLiftPosition();
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
            return new setLiftPosition();
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
            return new setLiftPosition();
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
            return new setLiftPosition();
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
            return new setLiftPosition();
        }

    }
    public class Claw {

    }
    FtcDashboard dash = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public abstract void runOpMode() throws InterruptedException;
}
