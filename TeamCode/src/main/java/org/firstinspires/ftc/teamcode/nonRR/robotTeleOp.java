package org.firstinspires.ftc.teamcode.nonRR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class robotTeleOp extends LinearOpMode {

    // Initialize the robot hardware.
    robotHardware robot = new robotHardware(this);

    // Code to run ONCE when the driver hits INIT.
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
//        robot.testMode();
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


//            clawControl();
//            angleControl();
//            extensionControl();
            driveControl();
            telemetry.addData("extension1 pos", robot.motorExtension1.getCurrentPosition());
            telemetry.addData("extension2 pos", robot.motorExtension2.getCurrentPosition());
            telemetry.addData("angle1 pos", robot.motorAngle1.getCurrentPosition());
            telemetry.addData("angle2 pos", robot.motorAngle2.getCurrentPosition());
            telemetry.update();
        }
    }

    boolean onOff = false;

//    private void clawControl() {
//        if (gamepad1.a) {
//            robot.claw.setPosition(1.0);
//        }
//        if (gamepad1.b) {
//            robot.claw.setPosition(0.6);
//        }
//
//    }

//    private void angleControl() {
//        if (gamepad1.dpad_up) {
//            robot.up();
//        }
//
//        if (gamepad1.dpad_down) {
//            robot.down();
//        }
//
//    }

    private void angleStepControl() {

    }

    private void angleControlWithTrigger() {

    }

//    private void extensionControl() {
//        if (gamepad1.left_bumper) {
//            robot.extend();
//        } else if (gamepad1.right_bumper) {
//            robot.retract();
//        }
//
//    }

    private void presetControl(){
        if (gamepad1.a){
            robot.setState(States.INITIAL, true);
        }
        if (gamepad1.b){
            robot.setState(States.CLIPINIT, true);
        }
        if (gamepad1.x){
            robot.setState(States.DROP, true);
        }
        if (gamepad1.y){
            robot.setState(States.CLIPFINAL, true);
        }
        if (gamepad1.left_bumper){
            robot.setState(States.PICKUP, true);
        }
        if (gamepad1.right_bumper){
            robot.setState(States.WALLPICKUP, true);
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

//        robot.claw.setPosition(gamepad1.left_trigger);
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;
        robot.startMove(drive, strafe, turn, scale);
    }
}


