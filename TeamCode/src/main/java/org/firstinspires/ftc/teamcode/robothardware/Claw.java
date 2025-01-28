package org.firstinspires.ftc.teamcode.robothardware;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Claw {
    private Servo claw;
    private Servo swing;
    private Servo angle;

    private TouchSensor limitSwitchAngle;
    private TouchSensor limitSwitchExtension;
    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("claw");
        swing = hardwareMap.servo.get("swing");

        angle = hardwareMap.servo.get("angle");

        limitSwitchAngle = hardwareMap.touchSensor.get("limitAngle");
        limitSwitchExtension = hardwareMap.touchSensor.get("limitExtension");

    }

}