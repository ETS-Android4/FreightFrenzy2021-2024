/*
 * Program Name:
 * Alliance:
 * Starting position
 * Functions of the program:
 *  - STEP1 =   gets the foundation into the build site
 *  - STEP2
 *
 *
 *
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Comp")
//@Disabled

public class TeleOp extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public TeleOp(){

    }   // end of BrokenBotTS constructor

    public void runOpMode(){
        double startTime;
        double timeElapsed;
        double v1, v2, v3, v4, robotAngle, powerLevel=1;
        double modePower = 1;
        double theta=0;
        double dpadup, dpaddown, dpadleft, dpadright;
        double r;
        double rightX, rightY;
        double peakPower = 0.30;
        boolean fieldCentric = true;
        double armPosition = 0.1;
        double linearServoPosition = 0.5;

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

        /*
         * Calibrate / initialize the gyro sensor
         */

        telemetry.addData("Z Value = ", drive.getZAngle());
        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            /*
             * Mecanum Drive Control section
             */
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
//                theta = robot.imu.getAngularOrientation().firstAngle;
            } else {
//                theta = 0;      // do not adjust for the angular position of the robot
            }
            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            rightY = -gamepad1.right_stick_y;
            r = -Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta)) + rightX + rightY) * powerLevel;
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta)) - rightX + rightY) * powerLevel;
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta)) + rightX + rightY) * powerLevel;
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta)) - rightX + rightY) * powerLevel;


          //  robot.servoLinear.setPosition(linearServoPosition);

            robot.motorLF.setPower(v1 * modePower );
            robot.motorRF.setPower(v2 * modePower );
            robot.motorLR.setPower(v3 * modePower );
            robot.motorRR.setPower(v4 * modePower );

            if(gamepad1.right_trigger>0){
                robot.motorTurnTable.setPower(-1 * gamepad1.right_trigger);
            }
            if(gamepad1.left_trigger>0){
                robot.motorTurnTable.setPower(1 * gamepad1.left_trigger);
            }
            if(gamepad1.right_bumper){
                robot.motorTurnTable.setPower(0);
            }
            
        //    telemetry.addData("Servo Position = ", robot.servoLinear.getPosition());
            telemetry.addData("motorLF = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("motorLR = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("motorRF = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("motorRR = ", robot.motorRR.getCurrentPosition());
        //    telemetry.addData("Shooter Encoder = ", robot.motorShooter.getCurrentPosition());
            //telemetry.addData("IMU Value: ", theta);
            telemetry.update();


        }   // end of while opModeIsActive()

    }   // end of runOpMode method
}   // end of BrokenBotTS.java class
