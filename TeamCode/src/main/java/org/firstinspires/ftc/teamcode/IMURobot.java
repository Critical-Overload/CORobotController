package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMURobot {

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    private BNO055IMU imu;

    private Orientation angles, lastAngles, startAngles;
    private double globalAngle;

    private double gain = 0.1;

    private LinearOpMode opMode;
    private Telemetry telemetry;

    public IMURobot(DcMotor motorFrontRight, DcMotor motorFrontLeft, DcMotor motorBackRight, DcMotor motorBackLeft,
                     BNO055IMU imu, LinearOpMode opMode){
        this.motorFrontRight = motorFrontRight;
        this.motorFrontLeft = motorFrontLeft;
        this.motorBackRight = motorBackRight;
        this.motorBackLeft = motorBackLeft;
        this.imu = imu;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        //this.telemetry = telemetry;
    }

    private BNO055IMU.Parameters getIMUParameters(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        return parameters;
    }

    private void setIMUParameters(){
        BNO055IMU.Parameters parameters = getIMUParameters();
        imu.initialize(parameters);
    }

    public void setupRobot() throws InterruptedException{
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setIMUParameters();

        while (!imu.isGyroCalibrated()) {
            telemetry.addData("IMU", "calibrating...");
            telemetry.update();
            Thread.sleep(50);
        }

        telemetry.addData("IMU", "ready");
        telemetry.update();
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        startAngles = lastAngles;
        globalAngle = 0;
    }

    private double getAngle(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    private void composeAngleTelemetry(){
        telemetry.addData("Start Angle", startAngles.firstAngle);
        telemetry.addData("Current Angle", angles.firstAngle);
        telemetry.addData("Global Angle", globalAngle);
    }

    public void completeStop(){
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void turnClockwise(double power){
        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
    }

    public void turnCounterClockwise(double power){
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(-power);
    }

    public void gyroTurn(int degrees, double power) throws InterruptedException{
        resetAngle();

        if(degrees < 0){
            turnClockwise(power);
        }else if(degrees > 0){
            turnCounterClockwise(power);
        }else{
            return;
        }

        if (degrees < 0){
            while (opMode.opModeIsActive() && getAngle() > degrees){
                composeAngleTelemetry();
                telemetry.addData("Target angle", degrees);
                telemetry.update();
            }
        }else{
            while (opMode.opModeIsActive() && getAngle() < degrees) {
                composeAngleTelemetry();
                telemetry.addData("Target angle", degrees);
                telemetry.update();
            }
        }

        completeStop();
        Thread.sleep(500);
        resetAngle();
    }

    private double getCorrection(){
        double angle = getAngle();
        double correction;

        if (angle == 0){
            correction = 0;
        }else{
            correction = -angle*gain;
        }
        telemetry.addData("Correction", correction);
        telemetry.update();

        return correction;
    }

    public void tankDrive(double leftPower, double rightPower){
        motorFrontLeft.setPower(leftPower);
        motorFrontRight.setPower(rightPower);
        motorBackLeft.setPower(leftPower);
        motorBackRight.setPower(rightPower);
    }

    public void gyroDrive(double power, int seconds) throws InterruptedException{
        resetAngle();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(timer.seconds() < seconds && opMode.opModeIsActive()){
            double correction = getCorrection();

            tankDrive(power + correction, power - correction);
        }
        completeStop();
        Thread.sleep(500);
        resetAngle();
    }

    public void setNewGain(double newGain){
        gain = newGain;
    }
}
