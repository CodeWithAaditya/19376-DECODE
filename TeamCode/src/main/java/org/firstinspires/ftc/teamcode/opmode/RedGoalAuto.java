package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedGoalAuto")
public class RedGoalAuto extends BaseGoalAuto{
    @Override
    protected boolean getAlliance() {
        return false; // RED
    }
}
