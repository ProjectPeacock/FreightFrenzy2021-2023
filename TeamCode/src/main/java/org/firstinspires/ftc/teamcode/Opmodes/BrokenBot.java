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

package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;
import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;

@TeleOp(name = "Broken Bot TS", group = "Programming Class")
//@Disabled

public class BrokenBot extends LinearOpMode {

    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;

    public BrokenBot(){

    }   // end of BrokenBot constructor

    public void runOpMode(){
        double startTime;
        double timeElapsed;
        double v1, v2, v3, v4, robotAngle, powerLevel=1;
        double modePower = 1;
        double theta;
        double r;
        double rightX, rightY;
        double buttonPress = 0;
        ElapsedTime currentTime = new ElapsedTime();
        boolean fieldCentric = false;
        double theta2 = 0;
        double dpaddown, dpadup, dpadright, dpadleft;

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);


        /*
         * Calibrate / initialize the gyro sensor
         */

        /*

        while (!robot.limitWobble.isPressed()){
            robot.motorWobbleLift.setPower(0.3);
        }
        robot.motorWobbleLift.setPower(0);

        robot.motorWobbleLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorWobbleLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.wobbleRotations(16, -0.5);

        double wobbleEncoder = robot.motorWobbleLift.getCurrentPosition();

         */

        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            /*
             * Mecanum Drive Control section
             */
            /*
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAngularOrientation().firstAngle;
            } else {
                theta = 0;      // do not adjust for the angular position of the robot
            }   // end of if(fieldCentric)
            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            rightY = -gamepad1.right_stick_y;
            r = -Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            v1 = (r * Math.cos(robotAngle + Math.toRadians(theta + theta2)) - rightX + rightY) * powerLevel;
            v2 = (r * Math.sin(robotAngle + Math.toRadians(theta + theta2)) + rightX + rightY) * powerLevel;
            v3 = (r * Math.sin(robotAngle + Math.toRadians(theta + theta2)) - rightX + rightY) * powerLevel;
            v4 = (r * Math.cos(robotAngle + Math.toRadians(theta + theta2)) + rightX + rightY) * powerLevel;

            robot.motorRF.setPower(Range.clip((v3 * modePower), -1, 1));
            robot.motorLF.setPower(Range.clip((v4 * modePower), -1, 1));
            robot.motorRR.setPower(Range.clip((v1 * modePower), -1, 1));
            robot.motorLR.setPower(Range.clip((v2 * modePower), -1, 1));

             */




            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAngularOrientation().firstAngle +90 ;
            } else {
                theta = 0;      // do not adjust for the angular position of the robot
            }
            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            rightY = -gamepad1.right_stick_y;
            r = -Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta)) + rightX + rightY) * powerLevel;
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta)) - rightX + rightY) * powerLevel;
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta)) + rightX + rightY) * powerLevel;
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta)) - rightX + rightY) * powerLevel;

            robot.motorLF.setPower(com.qualcomm.robotcore.util.Range.clip((v1 * modePower), -1, 1));
            robot.motorRF.setPower(com.qualcomm.robotcore.util.Range.clip((v2 * modePower), -1, 1));
            robot.motorLR.setPower(com.qualcomm.robotcore.util.Range.clip((v3 * modePower), -1, 1));
            robot.motorRR.setPower(com.qualcomm.robotcore.util.Range.clip((v4 * modePower), -1, 1));

            // Control which direction is forward and which is backward from the driver POV
            if (gamepad1.x && (currentTime.time() - buttonPress) > 0.3){
                if (theta2 == 180){
                    theta2 = 0;
                } else {
                    theta2 = 180;
                }
                buttonPress = currentTime.time();
            }   // end if (gamepad1.x && ...)

            if (gamepad2.dpad_down) {
                dpaddown = 1;
                telemetry.addData("Motor = ", "MotorLR");
            } else dpaddown = 0;

            if (gamepad2.dpad_up) {
                dpadup = 1;
                telemetry.addData("Motor = ", "MotorLF");
            } else dpadup = 0;

            if (gamepad2.dpad_right) {
                dpadright = 1;
                telemetry.addData("Motor = ", "MotorRR");
            } else dpadright = 0;

            if(gamepad2.dpad_left) {
                dpadleft = 1;
                telemetry.addData("Motor = ", "MotorRF");
            }
            else dpadleft = 0;

            if(gamepad1.right_trigger > 0){
                robot.motorDuck.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0){
                robot.motorDuck.setPower(-gamepad1.left_trigger);
            } else robot.motorDuck.setPower(0);

            robot.motorLF.setPower(v1 * modePower + dpadup);
            robot.motorRF.setPower(v2 * modePower + dpadleft);
            robot.motorLR.setPower(v3 * modePower + dpaddown);
            robot.motorRR.setPower(v4 * modePower + dpadright);


            telemetry.addData("MotorRF Encoder = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("MotorRR Encoder = ", robot.motorRR.getCurrentPosition());
            telemetry.addData("MotorLF Encoder = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("MotorLR Encoder = ", robot.motorLR.getCurrentPosition());

            telemetry.update();


        }   // end of while opModeIsActive()

    }   // end of runOpMode method
}   // end of BrokenBotTS.java class
