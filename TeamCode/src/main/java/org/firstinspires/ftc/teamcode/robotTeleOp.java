package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotHardware;

@TeleOp
public class robotTeleOp extends LinearOpMode {

    // Initialize the robot hardware.
    robotHardware robot = new robotHardware(this);

    // Code to run ONCE when the driver hits INIT.
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        robot.resetEncoders();
        waitForStart();


        // Run until the end of the match (driver presses STOP) or until stop is requested.
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (robot.motorAngle1.getCurrentPosition() >= 580) {
                robot.motorAngle1.setTargetPosition(580);
                robot.motorAngle2.setTargetPosition(580);
                robot.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            clawControl();
            angleControl();
            extensionControl();
            driveControl();
        }
    }

    boolean onOff = false;

    private void clawControl() {
        if (gamepad1.a) {
            robot.servoClaw.setPosition(1.0);
        }
        if (gamepad1.b) {
            robot.servoClaw.setPosition(0.6);
        }

    }

    private void angleControl() {
        if (gamepad1.dpad_up) {
            robot.up();
        }

        if (gamepad1.dpad_down) {
            robot.down();
        }

    }

    private void angleStepControl() {

    }

    private void angleControlWithTrigger() {

    }

    private void extensionControl() {
        if (gamepad1.left_bumper) {
            robot.extend();
        } else if (gamepad1.right_bumper) {
            robot.retract();
        }

    }

    private void testAngleControl() {
        //angle stuff
//        if (gamepad1.left_bumper){
//            robot.motorAngle1.setPower(0.7);
//            robot.motorAngle2.setPower(0.7);
//        } else {
//            robot.motorAngle1.setPower(0);
//            robot.motorAngle2.setPower(0);
//        }

    }

    private void testExtensionControl() {
        //extension stuff
//        if (gamepad1.right_bumper){
//            robot.motorExtension1.setPower(0.7);
//            robot.motorExtension2.setPower(0.7);
//        } else {
//            robot.motorExtension1.setPower(0);
//            robot.motorExtension2.setPower(0);
//        }
    }

    private void driveControl() {
        double scale = 0.6;
        if (gamepad1.left_trigger > 0.5) {
            gamepad1.rumble(500);
            scale = 0.9;
        }
//        else if (gamepad1.right_trigger > 0.5) {
//            scale = 0.1;
//        }
        robot.servoClaw.setPosition(1.0 - gamepad1.right_trigger);
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        robot.startMove(drive, strafe, turn, scale);

        robot.telemetryUpdate(telemetry);
    }

}


