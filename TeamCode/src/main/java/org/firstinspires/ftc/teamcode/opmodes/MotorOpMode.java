package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard;

@TeleOp
public class MotorOpMode extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard();

    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (board.isTouchSensorPressed()) {
            board.setMotorSpeed(0.4);
        } else {
            board.setMotorSpeed(0);
        }

        telemetry.addData("Motor rotations", board.getMotorRevolutions());
    }
}
