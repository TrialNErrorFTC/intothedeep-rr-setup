package org.firstinspires.ftc.teamcode.robothardware;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Lift {
        public DcMotorEx motorAngle1;
        public DcMotorEx motorAngle2;
        public DcMotorEx motorExtension1;
        public DcMotorEx motorExtension2;

    private OverflowEncoder motorAngle1Encoder;
    private OverflowEncoder motorAngle2Encoder;
    private OverflowEncoder motorExtension1Encoder;
    private OverflowEncoder motorExtension2Encoder;

        public Lift(@NonNull HardwareMap hardwareMap) {
            motorAngle1 = hardwareMap.get(DcMotorEx.class, "motorAngle1");


            motorAngle1.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorAngle1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorAngle1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorAngle1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorAngle1Encoder = new OverflowEncoder(new RawEncoder(motorAngle1));



            motorAngle2 = hardwareMap.get(DcMotorEx.class, "motorAngle2");


            motorAngle2.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorAngle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorAngle2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorAngle2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorAngle2Encoder = new OverflowEncoder(new RawEncoder(motorAngle2));


            motorExtension1 = hardwareMap.get(DcMotorEx.class, "motorExtension1");


            motorExtension1.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorExtension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorExtension1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorExtension1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorExtension1Encoder = new OverflowEncoder(new RawEncoder(motorExtension1));



            motorExtension2 = hardwareMap.get(DcMotorEx.class, "motorExtension2");


            motorExtension2.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorExtension2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorExtension2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorExtension2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorExtension2Encoder = new OverflowEncoder(new RawEncoder(motorExtension2));

        }
    }