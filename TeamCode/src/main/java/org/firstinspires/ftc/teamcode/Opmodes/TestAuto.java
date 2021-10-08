package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.RobotState;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMechanum;

@Autonomous(name = "Test Autonomous", group = "Programming Class")
//@Disabled


public class TestAuto extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;

    public TestAuto(){

    }   // end of TestAuto constructor

    public void runOpMode(){
        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);

        /*
         * Initialize the drive class
         */
        DriveMechanum drive = new DriveMechanum(robot, opMode);

        /*
         * Calibrate / initialize the game sensor
         */

    telemetry.addData("Z Value = ", drive.getZAngle());
    telemetry.addData("Robot state = ", "INITIALIZED");
    telemetry.update();

    waitForStart();

    if(opModeIsActive()){
        drive.robotCorrect(0.20, 90, 10);
        drive.motorsHalt();
    }   // end of if opModeIsActive()

    }// end of runOpMode constructor
}

