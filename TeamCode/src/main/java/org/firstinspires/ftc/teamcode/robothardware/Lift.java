package org.firstinspires.ftc.teamcode.robothardware;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Lift {
        private DcMotorEx motorAngle1;
        private DcMotorEx motorAngle2;
        private DcMotorEx motorExtension1;
        private DcMotorEx motorExtension2;
        
        public Lift(@NonNull HardwareMap hardwareMap) {
            motorAngle1 = hardwareMap.get(DcMotorEx.class, "motorAngle1");


            motorAngle1.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorAngle1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorAngle1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorAngle1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            motorAngle2 = hardwareMap.get(DcMotorEx.class, "motorAngle2");


            motorAngle2.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorAngle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorAngle2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorAngle2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            motorExtension1 = hardwareMap.get(DcMotorEx.class, "motorExtension1");


            motorExtension1.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorExtension1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorExtension1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            motorExtension2 = hardwareMap.get(DcMotorEx.class, "motorExtension2");


            motorExtension2.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorExtension2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorExtension2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }