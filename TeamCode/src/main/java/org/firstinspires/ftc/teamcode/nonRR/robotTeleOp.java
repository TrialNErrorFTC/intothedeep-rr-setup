package org.firstinspires.ftc.teamcode.nonRR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class robotTeleOp extends LinearOpMode {

    robotHardware robot = new robotHardware(this);

    enum ArmStates {
        INITIAL,
        PICKUP,
        DROP,
        EXTENSIONRESET,
        ANGLERESET,
        NONE
    }

    enum PickupSubStates {
        START,
        RETRACT_EXTENSION,
        MOVE_ANGLE,
        EXTEND_EXTENSION,
        COMPLETE
    }

    enum DropSubStates {
        START,
        RETRACT_EXTENSION,
        MOVE_ANGLE,
        EXTEND_EXTENSION,
        COMPLETE
    }

    enum InitialSubStates {
        START,
        RETRACT_EXTENSION,
        MOVE_ANGLE,
        COMPLETE
    }

    ArmStates currentState = ArmStates.INITIAL;
    ArmStates previousState = ArmStates.NONE;

    PickupSubStates pickupSubState = PickupSubStates.START;
    DropSubStates dropSubState = DropSubStates.START;
    InitialSubStates initialSubState = InitialSubStates.START;

    @Override
    public void runOpMode() throws InterruptedException {
        if (opModeInInit()) {
            robot.init();
            robot.setServoState(States.INITIAL);
            resetMotors();
        }

        waitForStart();

        while (opModeIsActive()) {
            // Handle state transitions
            if (currentState != previousState) {
                telemetry.addData("State Change", "From %s to %s", previousState, currentState);
                previousState = currentState;
                telemetry.update();
            }

            // Process the current state
            switch (currentState) {
                case PICKUP:
                    handlePickupStateFSM();
                    break;
                case DROP:
                    handleDropStateFSM();
                    break;
                case INITIAL:
                    handleInitialStateFSM();
                    break;
                case EXTENSIONRESET:
                    robot.zeroExtension();
                    currentState = ArmStates.NONE;
                    break;
                case ANGLERESET:
                    robot.zeroAngle();
                    currentState = ArmStates.NONE;
                    break;
                case NONE:
                default:
                    telemetry.addLine("No valid state or system idle.");
                    telemetry.update();
                    break;
            }

            // Handle controls - these now just set the state
            adjustmentControl();
            clawControl();
            driveControl();
            presetControl();

            // Debugging telemetry
            telemetry.addData("Extension 1 Pos", robot.motorExtension1.getCurrentPosition());
            telemetry.addData("Extension 2 Pos", robot.motorExtension2.getCurrentPosition());
            telemetry.addData("Angle 1 Pos", robot.motorAngle1.getCurrentPosition());
            telemetry.addData("Angle 2 Pos", robot.motorAngle2.getCurrentPosition());
            telemetry.update();
        }
    }

    private void resetMotors() {
        // Motor reset logic remains the same
        robot.motorExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorAngle1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorAngle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.zeroAngle();
        robot.motorAngle1.setTargetPosition(0);
        robot.motorAngle2.setTargetPosition(0);
        robot.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(robot.motorAngle1.isBusy() || robot.motorAngle2.isBusy()){
            telemetry.addLine("angle moving to drop");
            telemetry.update();
        }
        robot.zeroExtension();
        robot.motorExtension1.setTargetPosition(0);
        robot.motorExtension2.setTargetPosition(0);
        robot.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void handlePickupStateFSM() {
        switch (pickupSubState) {
            case START:
                // No need to check motor positions here as the FSM controls the progression
                robot.setServoState(States.DEFAULT);
                robot.setExtensionState(States.INITIAL);
                pickupSubState = PickupSubStates.RETRACT_EXTENSION;
                break;

            case RETRACT_EXTENSION:
                // Simplified condition for state progression
                robot.setAngleState(States.PICKUP);
                pickupSubState = PickupSubStates.MOVE_ANGLE;
                telemetry.addLine("Retracting extension...");
                break;

            case MOVE_ANGLE:
                // Simplified condition for state progression
                robot.setExtensionState(States.PICKUP);
                pickupSubState = PickupSubStates.EXTEND_EXTENSION;
                telemetry.addLine("Moving angle to pickup...");
                break;

            case EXTEND_EXTENSION:
                // Simplified condition for state progression
                robot.setServoState(States.PICKUP);
                pickupSubState = PickupSubStates.COMPLETE;
                telemetry.addLine("Extending to pickup...");
                break;

            case COMPLETE:
                currentState = ArmStates.NONE;
                pickupSubState = PickupSubStates.START;
                telemetry.addLine("Pickup state complete!");
                break;
        }
        telemetry.update();
    }

    private void handleDropStateFSM() {
        switch (dropSubState) {
            case START:
                // No need to check motor positions here as the FSM controls the progression
                robot.setServoState(States.DEFAULT);
                robot.setExtensionState(States.INITIAL);
                dropSubState = DropSubStates.RETRACT_EXTENSION;
                break;

            case RETRACT_EXTENSION:
                // Simplified condition for state progression
                robot.setAngleState(States.DROP);
                dropSubState = DropSubStates.MOVE_ANGLE;
                telemetry.addLine("Retracting extension...");
                break;

            case MOVE_ANGLE:
                // Simplified condition for state progression
                robot.setExtensionState(States.DROP);
                dropSubState = DropSubStates.EXTEND_EXTENSION;
                telemetry.addLine("Moving angle to drop...");
                break;

            case EXTEND_EXTENSION:
                // Simplified condition for state progression
                robot.setServoState(States.DROP);
                dropSubState = DropSubStates.COMPLETE;
                telemetry.addLine("Extending to drop...");
                break;

            case COMPLETE:
                currentState = ArmStates.NONE;
                dropSubState = DropSubStates.START;
                telemetry.addLine("Drop state complete!");
                break;
        }
        telemetry.update();
    }

    private void handleInitialStateFSM() {
        switch (initialSubState) {
            case START:
                // No need to check motor positions here as the FSM controls the progression
                robot.setServoState(States.DEFAULT);
                robot.setExtensionState(States.DEFAULT);
                initialSubState = InitialSubStates.RETRACT_EXTENSION;
                break;

            case RETRACT_EXTENSION:
                // Simplified condition for state progression
                robot.setAngleState(States.DEFAULT);
                initialSubState = InitialSubStates.MOVE_ANGLE;
                telemetry.addLine("Retracting extension...");
                break;

            case MOVE_ANGLE:
                // Simplified condition for state progression
                initialSubState = InitialSubStates.COMPLETE;
                telemetry.addLine("Moving angle to initial...");
                break;

            case COMPLETE:
                currentState = ArmStates.NONE;
                initialSubState = InitialSubStates.START;
                telemetry.addLine("Initial state complete!");
                break;
        }
        telemetry.update();
    }

    private void presetControl() {
        if (gamepad2.dpad_down) {
            currentState = ArmStates.EXTENSIONRESET;
        } else if (gamepad2.dpad_up) {
            currentState = ArmStates.ANGLERESET;
        } else if (gamepad1.square) {
            currentState = ArmStates.PICKUP;
        } else if (gamepad1.triangle) {
            currentState = ArmStates.DROP;
        } else if (gamepad1.touchpad) {
            currentState = ArmStates.INITIAL;
        }
    }

    private void clawControl() {
        if (gamepad1.left_bumper) {
            robot.clawOpen();
        }
        if (gamepad1.right_bumper) {
            robot.clawGrab();
        }
    }

    private void adjustmentControl() {
        if (gamepad2.cross) {
            robot.extend();
        }
        if (gamepad2.circle) {
            robot.retract();
        }
        if (gamepad2.triangle) {
            robot.up();
        }
        if (gamepad2.square) {
            robot.down();
        }
    }

    private void driveControl() {
        double scale = 0.6;
        if (gamepad1.left_trigger > 0.5) {
            scale = 0.9;
        }

        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        robot.startMove(drive, strafe, turn, scale);
    }
}
