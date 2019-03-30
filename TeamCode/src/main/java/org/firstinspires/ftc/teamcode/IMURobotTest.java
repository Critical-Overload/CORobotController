package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.IMURobot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "IMURobotTest")
public class IMURobotTest extends LinearOpMode {

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    private BNO055IMU imu;

    public void runOpMode() throws InterruptedException{
        //name the motors
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        motorBackLeft = hardwareMap.dcMotor.get("BL");

        imu = hardwareMap.get(BNO055IMU.class, "imu"); //what is this do?

        IMURobot robot = new IMURobot(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, imu, this); //what does this do?

        robot.setupRobot();//why do we do this, what does this do?

        waitForStart(); //wait for the game to start

        robot.gyroTurn(90, 0.3); //turn 90 degrees counterclockwise
        sleep(500);
        robot.gyroTurn(-90, 0.3); //turn 120 degrees clockwise
        //sleep(500); //wait
        //robot.gyroDrive(0.3, 5);//go forward, time limit 5 seconds
        //sleep(500);//wait
        //robot.tankDrive(0.3, 0.3);//set power to motors, does not set time limit
        //sleep(5000);//wait. This sets time limit for how long the above line runs
        //robot.completeStop();//stop
    }
}
