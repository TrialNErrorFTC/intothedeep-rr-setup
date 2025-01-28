package org.firstinspires.ftc.teamcode.nonRR;

public enum States {
    INITIAL(0, 0, 0.0, 0),
    DEFAULT(0, 0, 0.0, (double) 225/300),
    PICKUP(100, -1600, 0.8,  (double) 225/300),
    WALLPICKUP(100, 100, 0.5,  (double) 225/300),
    DROP(690, -4700, 0.0,  (double) 225/300),
    CLIPFINAL(200, 200, 0.35,  (double) 225/300),
    CLIPINIT(250, 250, 0.15,  (double) 225/300);

    public final int motorAnglePosition;
    public final int motorExtensionPosition;
    public final double swingPosition;
    public final double anglePosition;

    States(int motorAnglePosition, int motorExtensionPosition, double swingPosition, double anglePosition) {
        this.motorAnglePosition = motorAnglePosition;
        this.motorExtensionPosition = motorExtensionPosition;
        this.swingPosition = swingPosition;
        this.anglePosition = anglePosition;
    }
}
