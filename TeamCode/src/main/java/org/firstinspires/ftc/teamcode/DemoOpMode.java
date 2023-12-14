package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Zohra Demo OpMode", group="Iterative OpMode")
public class DemoOpMode extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Initialized");
    }
}
