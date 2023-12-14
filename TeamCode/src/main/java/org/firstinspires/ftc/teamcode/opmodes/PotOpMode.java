package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard;

@TeleOp
public class PotOpMode extends OpMode {
    ProgrammingBoard board = new ProgrammingBoard();

    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void loop() {
        board.setServoPosition(board.getPotAngle());
        telemetry.addData("Servo Position", board.getServoPosition());
        telemetry.addData("Pot angle", board.getPotAngle());
    }
}
