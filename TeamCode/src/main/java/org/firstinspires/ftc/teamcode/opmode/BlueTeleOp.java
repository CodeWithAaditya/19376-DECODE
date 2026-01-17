package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BlueTeleOp")
public class BlueTeleOp extends BaseTeleOp{
    @Override
    protected boolean getAlliance() {
        return true; // BLUE
    }
}
