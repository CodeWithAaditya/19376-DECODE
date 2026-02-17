package org.firstinspires.ftc.teamcode.opmode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Blue Goal Auto")
public class BlueGoalAuto extends BaseGoalAuto{
    @Override
    protected boolean getAlliance() {
        return true; // BLUE
    }
}
