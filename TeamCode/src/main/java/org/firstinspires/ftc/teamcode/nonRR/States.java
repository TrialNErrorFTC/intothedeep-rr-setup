package org.firstinspires.ftc.teamcode.nonRR;

public enum States {
    INIT(0, 0, 0.0, 0.0, 0.0),
    PICKUP(50, 50, 0.5, 0.5, 0.5),
    WALLPICKUP(100, 100, 1.0, 1.0, 1.0),
    DROP(150, 150, 0.75, 0.75, 0.75),
    CLIPFINAL(200, 200, 0.25, 0.25, 0.25),
    CLIPINIT(250, 250, 0.0, 0.0, 0.0);

    public final int motorAnglePosition;
    public final int motorExtensionPosition;
    public final double swingLeftPosition;
    public final double swingRightPosition;
    public final double anglePosition;

    States(int motorAnglePosition, int motorExtensionPosition, double swingLeftPosition, double swingRightPosition, double anglePosition) {
        this.motorAnglePosition = motorAnglePosition;
        this.motorExtensionPosition = motorExtensionPosition;
        this.swingLeftPosition = swingLeftPosition;
        this.swingRightPosition = swingRightPosition;
        this.anglePosition = anglePosition;
    }
}
