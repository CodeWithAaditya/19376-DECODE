package org.firstinspires.ftc.teamcode.opmode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Red Goal Auto")
public class RedGoalAuto extends BaseGoalAuto{
    @Override
    protected boolean getAlliance() {
        return false; // RED
    }
}
