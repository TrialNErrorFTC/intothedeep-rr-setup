package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.nonRR.robotHardware;

@TeleOp
public class testCode extends LinearOpMode {
    //Initialize the robot hardware.
    Servo claw;
    Servo swingLeft;
    Servo swingRight;

    // Code to run ONCE when the driver hits INIT.
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        claw = hardwareMap.servo.get("claw");
        swingLeft = hardwareMap.servo.get("swingLeft");
        swingRight = hardwareMap.servo.get("swingRight");

        swingLeft.setDirection(Servo.Direction.REVERSE);


        // Run until the end of the match (driver presses STOP) or until stop is requested.
        if (isStopRequested()) return;
        while (opModeIsActive()) {
//            if (robot.motorAngle1.getCurrentPosition() >= 580) {
//                robot.motorAngle1.setTargetPosition(580);
//                robot.motorAngle2.setTargetPosition(580);
//                robot.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//
//
//            clawControl();
//            angleControl();
//            extensionControl();
//            driveControl();
            if(gamepad1.a){
                claw.setPosition(0.35);
            }
            if(gamepad1.b){
                claw.setPosition(0.5);
            }
            if(gamepad1.x){
                swingRight.setPosition(0);
                swingLeft.setPosition(0);

            }
            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.update();
        }
    }
}
//
//    boolean onOff = false;
//
//    private void testControl(){
//        if (gamepad1.a){
//            robot.frontLeft.setTargetPosition(384);
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.frontLeft.setPower(1);
//        }
//        if (gamepad1.b){
//            robot.frontRight.setTargetPosition(384);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.frontRight.setPower(1);
//        }
//        if (gamepad1.x){
//            robot.rearLeft.setTargetPosition(384);
//            robot.rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rearLeft.setPower(1);
//        }
//        if (gamepad1.y){
//            robot.rearRight.setTargetPosition(384);
//            robot.rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rearRight.setPower(1);
//        }
////        if (gamepad1.b){
////            robot.frontRight.setTargetPosition(384);
////            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        }
////        if (gamepad1.x){
////            robot.frontLeft.setTargetPosition(384);
////            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        }
////        if (gamepad1.y){
////            robot.frontLeft.setTargetPosition(384);
////            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        }
//    }
//    private void clawControl() {
//        if (gamepad1.a) {
//            robot.claw.setPosition(1.0);
//        }
//        if (gamepad1.b) {
//            robot.claw.setPosition(0.6);
//        }
//
//    }
//
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
//
//    private void angleStepControl() {
//
//    }
//
//    private void angleControlWithTrigger() {
//
//    }
//
//    private void extensionControl() {
//        if (gamepad1.left_bumper) {
//            robot.extend();
//        } else if (gamepad1.right_bumper) {
//            robot.retract();
//        }
//
//    }
//
//    private void testAngleControl() {
//        //angle stuff
////        if (gamepad1.left_bumper){
////            robot.motorAngle1.setPower(0.7);
////            robot.motorAngle2.setPower(0.7);
////        } else {
////            robot.motorAngle1.setPower(0);
////            robot.motorAngle2.setPower(0);
////        }
//
//    }
//
//    private void testExtensionControl() {
//        //extension stuff
////        if (gamepad1.right_bumper){
////            robot.motorExtension1.setPower(0.7);
////            robot.motorExtension2.setPower(0.7);
////        } else {
////            robot.motorExtension1.setPower(0);
////            robot.motorExtension2.setPower(0);
////        }
//    }
//
//    private void driveControl() {
//        double scale = 0.6;
//        if (gamepad1.left_trigger > 0.5) {
//            gamepad1.rumble(500);
//            scale = 0.9;
//        }
////        else if (gamepad1.right_trigger > 0.5) {
////            scale = 0.1;
////        }
//        robot.claw.setPosition(1.0 - gamepad1.right_trigger);
//        double drive = gamepad1.left_stick_y;
//        double strafe = -gamepad1.left_stick_x;
//        double turn = -gamepad1.right_stick_x;
//        robot.startMove(drive, strafe, turn, scale);
//
////        robot.telemetryUpdate(telemetry);
//    }
//
//}
//

