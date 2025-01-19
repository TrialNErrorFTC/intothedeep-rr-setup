package org.firstinspires.ftc.teamcode.nonRR;

public class AutoCommon {
    int L = 0;
    int W = 0;
    int wheelRadius = 0;
    int ticksPerRev = 0;
    int current_x = 0;
    int current_y = 0;
    int current_rotation = 0;
    int constantFL = 0;
    int constantFR = 0;
    int constantBL = 0;
    int constantBR = 0;
    void move(int target_x, int target_y, int target_rotation){
        int delta_y = target_y - current_y;
        int delta_x = target_x - current_x;
        int delta_rotation = target_rotation - current_rotation;

        int distanceFL = (delta_y - delta_x - (delta_rotation * (L + W)));
        int distanceFR = (delta_y + delta_x + (delta_rotation * (L + W)));
        int distanceBL = (delta_y + delta_x - (delta_rotation * (L + W)));
        int distanceBR = (delta_y - delta_x + (delta_rotation * (L + W)));

        int ticks = (int) (distanceFL / (2 * Math.PI * wheelRadius) * ticksPerRev);


    }
}
