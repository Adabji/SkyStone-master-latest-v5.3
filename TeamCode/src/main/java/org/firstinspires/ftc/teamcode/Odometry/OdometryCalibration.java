package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class OdometryCalibration extends LinearOpMode {
    //Drive motors
    DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;
    //Odometry Wheels
    DcMotor intakeMotor2, intakeMotor1, intakeMotor3;

    //IMU Sensor
    BNO055IMU imu;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "right front", rbName = "right back", lfName = "left front", lbName = "left back";
    String verticalLeftEncoderName = "intake motor 2", verticalRightEncoderName = "intake motor 1", horizontalEncoderName = "intake motor 3";

    final double PIVOT_SPEED = 0.5;

    //The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    final double COUNTS_PER_INCH = 1141.94659527;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(intakeMotor1, intakeMotor2, intakeMotor3, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(getZAngle() < 180 && getZAngle() > -20 && opModeIsActive()){
            rightFrontWheel.setPower(-PIVOT_SPEED);
            rightBackWheel.setPower(-PIVOT_SPEED);
            leftFrontWheel.setPower(PIVOT_SPEED);
            leftBackWheel.setPower(PIVOT_SPEED);
            if(getZAngle() < 150) {
                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
            }else{
                setPowerAll(-PIVOT_SPEED/2, -PIVOT_SPEED/2, PIVOT_SPEED/2, PIVOT_SPEED/2);
            }

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.update();
        }

        //Stop the robot
        setPowerAll(0, 0, 0, 0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }
        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(intakeMotor2.getCurrentPosition()) + (Math.abs(intakeMotor1.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        horizontalTickOffset = intakeMotor3.getCurrentPosition()/Math.toRadians(getZAngle());

        //Write the constants to text files


        while(opModeIsActive()){

            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("intakeMotor3  Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", -intakeMotor2.getCurrentPosition());
            telemetry.addData("Vertical Right Position", intakeMotor1.getCurrentPosition());
            telemetry.addData("intakeMotor3  Position", intakeMotor3.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            //Update values
            telemetry.update();
        }
    }

    private void initHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        rightFrontWheel = hardwareMap.dcMotor.get(rfName);
        rightBackWheel = hardwareMap.dcMotor.get(rbName);
        leftFrontWheel = hardwareMap.dcMotor.get(lfName);
        leftBackWheel = hardwareMap.dcMotor.get(lbName);

        intakeMotor2 = hardwareMap.dcMotor.get(vlEncoderName);
        intakeMotor1 = hardwareMap.dcMotor.get(vrEncoderName);
        intakeMotor3  = hardwareMap.dcMotor.get(hEncoderName);

        rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();

    }

    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    private double getZAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }

    /**
     * Sets power to all four drive motors
     * @param rf power for right front motor
     * @param rb power for right back motor
     * @param lf power for left front motor
     * @param lb power for left back motor
     */
    private void setPowerAll(double rf, double rb, double lf, double lb){
        rightFrontWheel.setPower(rf);
        rightBackWheel.setPower(rb);
        leftFrontWheel.setPower(lf);
        leftBackWheel.setPower(lb);
    }

}
