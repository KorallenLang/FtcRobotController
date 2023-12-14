package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class RobotHardware {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor objects  (Make them private so they can't be accessed externally)
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackRight = null;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        motorFrontLeft  = myOpMode.hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackLeft  = myOpMode.hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight  = myOpMode.hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight  = myOpMode.hardwareMap.get(DcMotor.class, "motorBackRight");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
//        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
//        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
//        motorBackRight.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    //TODO: Update documentation
    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     */
    public void driveRobot(double axial, double lateral, double yaw) {
        double max;
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial - lateral - yaw;
        double rightFrontPower = axial + lateral + yaw;
        double leftBackPower   = axial + lateral - yaw;
        double rightBackPower  = axial - lateral + yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    //TODO: Update documentation
    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     */
    public void setDrivePower(double leftFrontPower_, double leftBackPower_, double rightFrontPower_, double rightBackPower_) {
        // Output the values to the motor drives.
        motorFrontLeft.setPower(leftFrontPower_);
        motorFrontRight.setPower(rightFrontPower_);
        motorBackLeft.setPower(leftBackPower_);
        motorBackRight.setPower(rightBackPower_);
    }
}
