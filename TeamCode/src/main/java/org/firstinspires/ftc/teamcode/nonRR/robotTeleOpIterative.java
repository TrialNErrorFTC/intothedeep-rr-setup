
package org.firstinspires.ftc.teamcode.nonRR;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class robotTeleOpIterative extends OpMode {
    // Initialize the robot hardware.
    robotHardware robot = new robotHardware(this);

    // Code to run ONCE when the driver hits INIT.
    @Override
    public void init() {
        robot.init();
        robot.setServoState(States.INITIAL);
        robot.motorExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorAngle1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorAngle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.zeroExtension();
        robot.zeroAngle();
        robot.testMode();
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY.
    @Override
    public void init_loop() {
    }

    // Code to run ONCE when the driver hits PLAY.
    @Override
    public void start() {
        robot.setState(States.INITIAL, true);
    }
    private void presetControl(){
//        if (gamepad1.a){
//            robot.setState(States.INIT);
//        }
//        if (gamepad1.b){
//            robot.setState(States.CLIPINIT);
//        }

//        if (gamepad1.right_bumper){
//            robot.setState(States.WALLPICKUP);
//        }
    }

    private void testControl(){
        if (gamepad1.left_bumper){
            robot.swingLeft.setPosition(gamepad1.right_trigger);
            robot.swingRight.setPosition(gamepad1.right_trigger);

        }
        if (gamepad1.right_bumper){
            robot.angle.setPosition(gamepad1.right_trigger);
        }
    }


    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP.
    @Override
    public void loop() {
//        Gamepad currentGamepad1 = new Gamepad();
//        Gamepad currentGamepad2 = new Gamepad();
//
//        Gamepad previousGamepad1 = new Gamepad();
//        Gamepad previousGamepad2 = new Gamepad();

        if(gamepad1.dpad_down){
            telemetry.addLine("Caution: Please set the arm to the correct position(all the way down) before zeroing the extension. Press dpad_left to confirm.");
            telemetry.update();
            if(gamepad1.dpad_left){
                robot.zeroExtension();
            }
        }

        if(gamepad1.dpad_up){
            telemetry.addLine("Zeroing the angle. Please make sure the extension is all the way down before proceeding. Press dpad_right to confirm.");
            telemetry.update();
            if(gamepad1.dpad_right){
                robot.zeroAngle();
            }
        }

        if (gamepad1.x){
//            robot.setExtensionState(States.INITIAL);
//            while(robot.motorExtension1.isBusy() || robot.motorExtension2.isBusy()){
//                telemetry.addLine("extension moving back to inital");
//                telemetry.update();
//            }
//            robot.setAngleState(States.DROP);
//            while(robot.motorAngle1.isBusy() || robot.motorAngle2.isBusy()){
//                telemetry.addLine("angle moving to drop");
//                telemetry.update();
//            }
//            robot.setExtensionState(States.DROP);
            robot.setServoState(States.DROP);
            //180 degrees swing arm
        }
        if (gamepad1.y){
//            robot.setExtensionState(States.INITIAL);
//            while(robot.motorExtension1.isBusy() || robot.motorExtension2.isBusy()){
//                telemetry.addLine("extension moving back to inital");
//                telemetry.update();
//            }
//            robot.setAngleState(States.PICKUP);
//            while(robot.motorAngle1.isBusy() || robot.motorAngle2.isBusy()){
//                telemetry.addLine("angle moving to pickup");
//                telemetry.update();
//            }
//            robot.setExtensionState(States.PICKUP);
            robot.setServoState(States.PICKUP);
            //180 degrees swing arm
        }
        if(gamepad1.left_bumper){
            robot.setServoState(States.INITIAL);
        }
ER


//        if (gamepad1.y){
//            robot.setState(States.CLIPFINAL);
//        }
        if(gamepad1.a){
            robot.extend(); //this should retract
        }
        if(gamepad1.b){
            robot.retract(); //this should retract
        }

//        if (robot.motorAngle1.getCurrentPosition() >= 580) {
//            robot.motorAngle1.setTargetPosition(580);
//            robot.motorAngle2.setTargetPosition(580);
//            robot.motorAngle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.motorAngle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }

        driveControl();
        presetControl();
        telemetry.addData("extension1 pos", robot.motorExtension1.getCurrentPosition());
        telemetry.addData("extension2 pos", robot.motorExtension2.getCurrentPosition());
        telemetry.addData("angle1 pos", robot.motorAngle1.getCurrentPosition());
        telemetry.addData("angle2 pos", robot.motorAngle2.getCurrentPosition());

        telemetry.addData("claw pos", robot.claw.getPosition());
        telemetry.addData("angle pos", robot.angle.getPosition());
        telemetry.addData("swingLeft pos", robot.swingLeft.getPosition());
        telemetry.addData("swingRight pos", robot.swingRight.getPosition());

        telemetry.update();
    }

    boolean onOff = false;

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
