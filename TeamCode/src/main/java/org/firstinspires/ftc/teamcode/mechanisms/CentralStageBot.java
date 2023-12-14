package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CentralStageBot {
    private OpMode myOpMode = null;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public CentralStageBot(OpMode opMode) {
        myOpMode = opMode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    public void init() {
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "rightBackDrive");
    }
}
