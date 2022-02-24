package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@Autonomous(name = "Remote Auto - Red", group = "Leviathan")
//@Disabled

public class LeviathanAutoRed extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;
    private State state = State.TURNTABLE;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    public LeviathanAutoRed(){

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
                    double elementDistance = robot.sensorWall.getDistance(DistanceUnit.INCH);
                    telemetry.addData("element Distance = ", elementDistance);
                    telemetry.update();
                // Drive off wall
                    drive.robotCorrect(0.5, 180, 0.5);
                // Rotate 90
                    drive.PIDRotate(-90,0.3);
                // Drive to wall, fast and then slow
                    drive.robotCorrect(0.5, 180, 1.0);
                    drive.robotCorrect(0.25, 180, 0.8);
                // Strafe into Turn table
                    drive.driveSensorDistance(0.25, 90, 6.8);


                // Drive back to the wall at a very slow rate
                    drive.robotCorrect(0.1,180,0.1);
                // Run turn table motor, drop duck
                    robot.motorTurnTable.setPower(-0.2);

                    sleep(5000);

                    robot.motorTurnTable.setPower(0);
                // Back away from wall
                    drive.robotCorrect(0.3,0,0.1);

                    // Strafe into parking spot
                    drive.driveSensorDistanceOut(0.25, -90, 24);

                // Stop
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

