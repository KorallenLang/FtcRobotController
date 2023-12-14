package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard;

@Autonomous
public class AutoOpMode extends OpMode {
    enum State {
        START,
        SECOND_STEP,
        THIRD_STEP,
        DONE
    }

    ProgrammingBoard board = new ProgrammingBoard();
    State state = State.START;
    double lastTime;

    @Override
    public void init() {
        board.init(hardwareMap);
    }

    @Override
    public void start() {
        state = State.START;
        resetRuntime();
        lastTime = getRuntime();
    }

    @Override
    public void loop() {
        telemetry.addData("State", state);
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Time in state", getRuntime() - lastTime);

        board.setMotorSpeed(0.25);
        switch(state) {
            case START:
                if (getRuntime() >= 1.25) {
                    board.setMotorSpeed(0.5);
                    state = State.SECOND_STEP;
                    lastTime = getRuntime();
                }
                break;
            case SECOND_STEP:
                if (getRuntime() >= lastTime + 1) {
                    board.setMotorSpeed(0.75);
                    state = State.THIRD_STEP;
                    lastTime = getRuntime();
                }
                break;
            case THIRD_STEP:
                if (getRuntime() >= lastTime + 0.75) {
                    board.setMotorSpeed(1);
                    state = State.DONE;
                    lastTime = getRuntime();
                }
                break;
            default:
                telemetry.addData("Auto", "Finished");
        }
    }
}
