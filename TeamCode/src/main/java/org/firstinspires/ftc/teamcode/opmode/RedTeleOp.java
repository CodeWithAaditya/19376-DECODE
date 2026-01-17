package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RedTeleOp")
public class RedTeleOp extends BaseTeleOp{
    @Override
    protected boolean getAlliance() {
        return false; // RED
    }
}
