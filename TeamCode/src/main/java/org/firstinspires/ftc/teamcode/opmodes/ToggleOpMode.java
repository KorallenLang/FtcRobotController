package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard;

@TeleOp
public class ToggleOpMode extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard();
    boolean alrPressed;
    boolean motorOn;

    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a && !alrPressed) {
            motorOn = !motorOn;
            telemetry.addData("Motor status", motorOn);
            if (motorOn) {
                board.setMotorSpeed(0.5);
            } else {
                board.setMotorSpeed(0);
            }
        }
        alrPressed = gamepad1.a;
    }
}
