package org.firstinspires.ftc.teamcode.nonRR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoPush extends LinearOpMode {

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
            robot.frontLeft.setPower(-0.5);
            robot.rearRight.setPower(-0.5);
            robot.frontRight.setPower(0.5);
            robot.rearLeft.setPower(0.5);
            sleep(2000);
            robot.frontLeft.setPower(0);
            robot.rearRight.setPower(0);
            robot.frontRight.setPower(0);
            robot.rearLeft.setPower(0);
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

}


