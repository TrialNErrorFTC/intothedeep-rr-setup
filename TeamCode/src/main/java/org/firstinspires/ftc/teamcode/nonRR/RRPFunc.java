package org.firstinspires.ftc.teamcode.nonRR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class RRPFunc extends LinearOpMode {

    robotHardware robot = new robotHardware(this);

    enum ArmStates {
        INITIAL,
        PICKUP,
        DROP,
        EXTENSIONRESET,
        ANGLERESET,
        NONE,
        MANUALCONTROL
    }

    ArmStates currentState = ArmStates.INITIAL;
    ArmStates previousState = ArmStates.NONE;

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
                    handlePickupState();
                    currentState = ArmStates.NONE;
                    break;
                case DROP:
                    handleDropState();
                    currentState = ArmStates.NONE;
                    break;
                case INITIAL:
                    handleInitialState();
                    currentState = ArmStates.NONE;
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
                    break;
                case MANUALCONTROL:
                    handleManualControl();
                    break;
                default:
                    break;
            }

            // Handle controls - these now just set the state
            adjustmentControl();
            presetControl();
            clawControl();
            driveControl();
            // Debugging telemetry
            telemetry.addData("Extension 1 Pos", robot.motorExtension1.getCurrentPosition());
            telemetry.addData("Extension 2 Pos", robot.motorExtension2.getCurrentPosition());
            telemetry.addData("Angle 1 Pos", robot.motorAngle1.getCurrentPosition());
            telemetry.addData("Angle 2 Pos", robot.motorAngle2.getCurrentPosition());
            telemetry.addData("Current State", currentState);
            telemetry.update();
        }
    }


    private void resetMotors() {
        robot.motorExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorAngle1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorAngle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.zeroAngle();

        robot.motorAngle1.setTargetPosition(0);
        robot.motorAngle2.setTargetPosition(0);
        robot.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.motorAngle1.isBusy() || robot.motorAngle2.isBusy()) {
            telemetry.addLine("Angle moving to reset");
            telemetry.update();

        }

        robot.zeroExtension();
        robot.motorExtension1.setTargetPosition(0);
        robot.motorExtension2.setTargetPosition(0);
        robot.motorExtension1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorExtension2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private void handlePickupState() {
        robot.setServoState(States.DEFAULT);
        robot.setExtensionState(States.INITIAL);

        // First phase: Retracting extension
        while (robot.motorExtension1.isBusy() || robot.motorExtension2.isBusy()) {
            telemetry.addLine("Retracting extension...");
            driveControl();
            clawControl();
            presetControl();
            angleControl();

            // If the state changes, stop all motors for extension and angle
            if (currentState != ArmStates.PICKUP) {
                robot.motorExtension1.setPower(0);
                robot.motorExtension2.setPower(0);
                robot.motorAngle1.setPower(0);
                robot.motorAngle2.setPower(0);
                return; // Exit the method
            }

            telemetry.update();
        }

        // Second phase: Moving angle to pickup position
        robot.setAngleState(States.PICKUP);
        while (robot.motorAngle1.isBusy() || robot.motorAngle2.isBusy()) {
            telemetry.addLine("Moving angle to pickup...");
            driveControl();
            clawControl();
            presetControl();
            angleControl();

            // If the state changes, stop all motors for extension and angle
            if (currentState != ArmStates.PICKUP) {
                robot.motorExtension1.setPower(0);
                robot.motorExtension2.setPower(0);
                robot.motorAngle1.setPower(0);
                robot.motorAngle2.setPower(0);
                return; // Exit the method
            }

            telemetry.update();
        }

        // Third phase: Extending to pickup position
        robot.setExtensionState(States.PICKUP);
        while (robot.motorExtension1.isBusy() || robot.motorExtension2.isBusy()) {
            telemetry.addLine("Extending to pickup...");
            driveControl();
            clawControl();
            presetControl();
            angleControl();

            // If the state changes, stop all motors for extension and angle
            if (currentState != ArmStates.PICKUP) {
                robot.motorExtension1.setPower(0);
                robot.motorExtension2.setPower(0);
                robot.motorAngle1.setPower(0);
                robot.motorAngle2.setPower(0);
                return; // Exit the method
            }

            telemetry.update();
        }

        // Final phase: Setting the servo to pickup state
        robot.setServoState(States.PICKUP);
        telemetry.addLine("Pickup state complete!");
        telemetry.update();
    }

    private void handleDropState() {
        robot.setServoState(States.DEFAULT);
        robot.setExtensionState(States.INITIAL);

        while (robot.motorExtension1.isBusy() || robot.motorExtension2.isBusy()) {
            telemetry.addLine("Retracting extension...");
            driveControl();
            clawControl();
            presetControl();
            angleControl();

            //if another preset is played is pressed stop the code
            if(currentState != ArmStates.DROP){

                //stops the code
                robot.motorAngle1.setPower(0);
                robot.motorAngle2.setPower(0);
                robot.motorExtension1.setPower(0);
                robot.motorExtension2.setPower(0);
                return;

            }
            telemetry.update();

        }

        robot.setAngleState(States.DROP);
        while (robot.motorAngle1.isBusy() || robot.motorAngle2.isBusy()) {
            telemetry.addLine("Moving angle to drop...");
            driveControl();
            clawControl();
            presetControl();
            angleControl();

            //if another preset is played is pressed stop the code
            if(currentState != ArmStates.DROP){

                //stops the code
                robot.motorAngle1.setPower(0);
                robot.motorAngle2.setPower(0);
                robot.motorExtension1.setPower(0);
                robot.motorExtension2.setPower(0);
                return;

            }
            telemetry.update();

        }

        robot.setExtensionState(States.DROP);
        while (robot.motorExtension1.isBusy() || robot.motorExtension2.isBusy()) {
            telemetry.addLine("Extending to drop...");
            driveControl();
            clawControl();
            presetControl();
            angleControl();

            //if another preset is played is pressed stop the code
            if(currentState != ArmStates.DROP){

                //stops the code
                robot.motorAngle1.setPower(0);
                robot.motorAngle2.setPower(0);
                robot.motorExtension1.setPower(0);
                robot.motorExtension2.setPower(0);
                return;

            }
            telemetry.update();

        }

        robot.setServoState(States.DROP);
        telemetry.addLine("Drop state complete!");
        telemetry.update();
    }
    private void angleControl() {
        double minAngle = 0.0; // Minimum angle for the claw servo
        double maxAngle = 180.0; // Maximum angle for the claw servo (change to 300.0 if needed)
        double angleStep = 5.0; // Step size for increment/decrement in degrees

        // Static variable to store the current angle
        if (!robot.servoAngleInitialized) {
            robot.currentAngle = (minAngle + maxAngle) / 2; // Start at midpoint
            robot.servoAngleInitialized = true;
        }

        // Increment angle with one button, decrement with another
        if (gamepad2.dpad_left && robot.currentAngle <= maxAngle - angleStep) {
            robot.currentAngle += angleStep;
        } else if (gamepad2.dpad_right && robot.currentAngle >= minAngle + angleStep) {
            robot.currentAngle -= angleStep;
        }

        // Normalize the angle to a servo position (0.0 to 1.0)
        double servoPosition = robot.currentAngle / maxAngle;

        // Set the servo positions
        robot.angle.setPosition(servoPosition);

        // Telemetry for debugging
        telemetry.addData("Current Angle", robot.currentAngle);
        telemetry.addData("Servo Position", servoPosition);
        telemetry.update();
    }

    private void handleInitialState() {
        robot.setServoState(States.DEFAULT);
        robot.setExtensionState(States.DEFAULT);

        while (robot.motorExtension1.isBusy() || robot.motorExtension2.isBusy()) {
            telemetry.addLine("Retracting extension...");
            driveControl();
            clawControl();
            presetControl();
            angleControl();

            //if another preset is played is pressed stop the code
            if(currentState != ArmStates.INITIAL){
                //stops the code
                robot.motorAngle1.setPower(0);
                robot.motorAngle2.setPower(0);
                robot.motorExtension1.setPower(0);
                robot.motorExtension2.setPower(0);
                return;

            }
            telemetry.update();

        }

        robot.setAngleState(States.DEFAULT);
        while (robot.motorAngle1.isBusy() || robot.motorAngle2.isBusy()) {
            telemetry.addLine("Moving angle to initial...");
            driveControl();
            clawControl();
            presetControl();
            angleControl();

            //if another preset is played is pressed stop the code
            if(currentState != ArmStates.INITIAL){

                //stops the code
                robot.motorAngle1.setPower(0);
                robot.motorAngle2.setPower(0);
                robot.motorExtension1.setPower(0);
                robot.motorExtension2.setPower(0);
                return;

            }
            telemetry.update();
        }

        telemetry.addLine("Initial state complete!");
        telemetry.update();
    }

    private void handleManualControl() {
        if(currentState != ArmStates.MANUALCONTROL){

            //stops the code
            robot.motorAngle1.setPower(0);
            robot.motorAngle2.setPower(0);
            robot.motorExtension1.setPower(0);
            robot.motorExtension2.setPower(0);
            return;

        } else if (gamepad2.cross) {
            robot.extend();
        } else if (gamepad2.circle) {
            robot.retract();
        } else if (gamepad2.triangle) {
            robot.up();
        } else if (gamepad2.square) {
            robot.down();
        }

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
        } else if (gamepad1.dpad_left) {
            currentState = ArmStates.MANUALCONTROL;
            telemetry.addData("I am in",currentState);
            telemetry.addLine();

        }
    }

    private void clawControl() {
        if (gamepad1.left_bumper) {
            robot.clawOpen();
        }
        if (gamepad1.right_bumper) {
            robot.clawGrab();
        }
        if (gamepad2.left_bumper) {
            robot.clawOpen();
        }
        if (gamepad2.right_bumper) {
            robot.clawGrab();
        }
    }

    private void adjustmentControl() {
        if (gamepad1.dpad_right) {
            currentState = ArmStates.MANUALCONTROL;
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
