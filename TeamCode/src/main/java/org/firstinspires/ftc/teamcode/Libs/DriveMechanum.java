package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HWProfile.HWProfile;

public class DriveMechanum {

    private HWProfile robot;
    public double RF, LF, LR, RR;
    public LinearOpMode opMode;

    /*
     * Constructor method
     */
    public DriveMechanum(HWProfile myRobot, LinearOpMode myOpMode){
       robot = myRobot;
       opMode = myOpMode;
    }  // closes DriveMechanum constructor Method

    public void robotCorrect(double power, double heading, double duration) {
        String action = "Initializing";
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double theta = Math.toRadians(90 + heading);
        ElapsedTime runtime = new ElapsedTime();

        if(runtime.time() >= duration) active = false

        updateValues(action, initZ, theta, currentZ, zCorrection);

        while(opMode.opModeIsActive() && active){
            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            if(runtime.time() >= duration) active = false;

            currentZ = getZAngle();
            if (currentZ != initZ){
                zCorrection = Math.abs(initZ - currentZ);

                if (heading > 180 && heading < 359.999999){

                }   // end of if heading > 180 && heading < 359.999999

                /*
                 * Current vid stopping point at 57:06
                 */


            } // end of if current != initZ

            /*
             * Apply power to the drive wheels
             */
            robot.motorRF.setPower(RF);
            robot.motorLF.setPower(LF);
            robot.motorLR.setPower(LR);
            robot.motorRR.setPower(RR);
        }   // end of while loop

        motorsHalt();


    }   // close robotCorrect method

    /*
     * Method getZAngle()
     */
    public double getZAngle(){
        return (-robot.imu.getAngularOrientation().firstAngle)
    }   // close getZAngle method


    /*
     * Method motorsHalt
     */
    public void  motorsHalt(){
        robot.motorRF.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    } // end of motorsHalt method


    /*
     * Method updateValues
     */
    public void updateValues(String action, double initZ, double theta, double currentZ, double zCorrection){
        opMode.telemetry.addData("Current Action = ", action);
        opMode.telemetry.addData("InitZ value = ", initZ);
        opMode.telemetry.addData("Theta Value = ", theta);
        opMode.telemetry.addData("Current Z value = ", currentZ);
        opMode.telemetry.addData("zCorrection Value = ", zCorrection);

        opMode.telemetry.addData("Right Front = ", RF);
        opMode.telemetry.addData("Left Front = ", LF);
        opMode.telemetry.addData("Left Rear = ", LR);
        opMode.telemetry.addData("Right Rear = ", RR);
        opMode.telemetry.update();
    }   // close updateValues method

}



