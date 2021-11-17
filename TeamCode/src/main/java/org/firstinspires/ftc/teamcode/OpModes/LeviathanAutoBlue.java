package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

import java.util.List;

@Autonomous(name = "Remote Auto - Blue", group = "Leviathan")
//@Disabled

public class LeviathanAutoBlue extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;
    private State state = State.TURNTABLE;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    public LeviathanAutoBlue(){

    }   // end of TestAuto constructor

    public void runOpMode(){
        double startTime = 0;
        double timeElapsed;
        double armPosition;
        int position = 1;
        ElapsedTime runTime = new ElapsedTime();

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);

        /*
         * Initialize the drive class
         */
        DriveMecanum drive = new DriveMecanum(robot, opMode);

        // Let the user know the robot is initialized
        telemetry.addData("Z Value = ", drive.getZAngle());
        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        waitForStart();
        startTime = runTime.time();

        while(opModeIsActive()) {

            switch (state) {
                case TEST:

                    state = State.HALT;

                    break;

                case RING_DETECT:

                    state = State.HALT;

                    break;

                case PATH_DECISION:

                    state = State.HALT;

                    break;

                case TURNTABLE:
                    // Strafe diagonally towards outside wall - avoids rings on the field
                    drive.robotCorrect(0.5, 0, 1.4);

                    drive.PIDRotate(-90,0.3);
                    // Drive to target zone A
                    drive.robotCorrect(0.5, 0, 3);

                    // Strafe away from wall to place the wobble goal
                    drive.robotCorrect(0.5, -90, 1);

                    robot.motorTurnTable.setPower(0.1);

                    sleep(5000);

                    robot.motorTurnTable.setPower(0);

                    drive.robotCorrect(0.5,90,2);
                    // Place wobble goal
                    // drive.placeWobbleGoal();

                    // Return to starting position
                    // Drive back to avoid hitting the wobble goal
                    drive.driveDistance(0.5, 180, 18);

                    // Strafe towards the outside wall - avoiding rings on the field
                    drive.robotCorrect(0.5, -135, 0.75);

                    // Drive towards the starting position to get reset
                    drive.driveDistance(0.5, 180, 36);
                    state = State.HALT;

                    break;

                case HALT:

                    // Stop all motors
                    drive.motorsHalt();

                    // End the program
                    requestOpModeStop();

                    break;

            }   // end of the switch state

        }   // end of while opModeIsActive()

    }   // end of runOpMode method

    /*
     * Enumerate the states of the machine
     */
    enum State {
        TEST, RING_DETECT, PATH_DECISION, TURNTABLE, WOBBLE1B, WOBBLE1C, RESET_START, WOBBLE2A, WOBBLE2B, WOBBLE2C, PARK, PREP_SHOOTER, SHOOT, HALT;
    }   // end of enum State

    /**
     * Initialize the Vuforia localization engine.
     */
}   // end of TestAuto.java class

