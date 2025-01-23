package org.firstinspires.ftc.teamcode.nonRR;

public enum States {
    INITIAL(0, 0, 0.0, 0.0, 0),
    PICKUP(50, -1600, 0.7, 0.7, (double) 180 /300),
    WALLPICKUP(100, 100, -0.25, 0.25, (double) 180 /300),
    DROP(690, -3500, 0.0, 0.0, 0.0),
    CLIPFINAL(200, 200, 0.35, 0.35, (double) 180 /300),
    CLIPINIT(250, 250, 0.15, 0.15, (double) 180 /300);

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
