package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.ArrayList;

@Autonomous(name="AutonomousV1", group="Robot")
public class AutoV1 extends LinearOpMode {

    /* Declare All: Drive & Arm Motors, Wrist Servo, Sensors */

    // Declare OpMode members.
    private DcMotor FrontRight = null;
    private DcMotor BackRight = null;
    private DcMotor BackLeft = null;
    private DcMotor FrontLeft = null;

    // New motors
    private DcMotor Extend = null;
    private DcMotor Arm = null;
    private Servo Wrist = null;
    private Servo Intake = null;
    
    // Sensors
    private BHI260IMU imu;

    /* Constants */
    
    //Global Variables
    private ElapsedTime runtime = new ElapsedTime();
    double headingError;
    
    //Motor Constants
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.1;
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    
    //Speed Constants
    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.6;
    
    //Timeouts
    static final double DEFAULT_TIMEOUT = 5;
    static final long INPUT_DELAY = 0;
    
    //Arm Constants
    static final int OPEN_ANGLE = 140;
    static final int CLOSE_ANGLE = 210;
    static final double BUCKET_SPEED = 1;
    static final double BUCKET_DIS = 77;
    static final double BUCKET_TIMEOUT = 1.45;
    
    @Override
    public void runOpMode() {
            
        /* Initialize All: IMU, Drive & Arm */
        
        // Drive motors
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");

        // New motors
        Extend = hardwareMap.get(DcMotor.class, "Extend");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Intake = hardwareMap.get(Servo.class, "Intake");
        
        // Initialize IMU
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
        
        /* Setup Motors */
        
        // Reset drive encoders
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset new motor encoders
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // All Motor Directions
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        Arm.setDirection(DcMotor.Direction.FORWARD);
        Extend.setDirection(DcMotor.Direction.REVERSE);
        
        waitForStart();
        
        imu.resetYaw();
        
        /* Actions */
        
        // 1: Drive Forward, (distance, speed(optional))
        // 2: Turn, (heading, speed(optional))
        
        
    }
    
    /* Functions */
    
    // Action Decoder
    public void action(int type, Double... value) {
        // Drive Forward
        if (type == 1) {
            double a = value[0];
            if (value.length > 1) {
                double b = value[1];
                rawSpeedEncoder(b, 0, 0, a, a, a, a, 0, 0, DEFAULT_TIMEOUT);
            } else {
                rawSpeedEncoder(1, 0, 0, a, a, a, a, 0, 0, DEFAULT_TIMEOUT);
            }
        }
        // Turn To Heading
        if (type == 2) {
            double a = value[0];
            if (value.length > 1) {
                double b = value[1];
                turnToHeading(b, a);
            } else {
                turnToHeading(1, a);
            }
        }
    }
    
    // All Motor Encoders
    public void rawSpeedEncoder(double DriveSpeed,
                                double ExtendSpeed,
                                double ArmSpeed,
                             double FrontleftInches,
                             double FrontrightInches,
                             double BackrightInches,
                             double BackleftInches,
                             double ExtendInches,
                             double ArmInches,
                             double timeoutS) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        int newExtendTarget;
        int newArmTarget;


        if (opModeIsActive()) {

            newFrontLeftTarget =
                    FrontLeft.getCurrentPosition() +
                    (int)(FrontleftInches * COUNTS_PER_INCH);

            newFrontRightTarget =
                    FrontRight.getCurrentPosition() +
                    (int)(FrontrightInches * COUNTS_PER_INCH);

            newBackLeftTarget =
                    BackLeft.getCurrentPosition() +
                    (int)(BackleftInches * COUNTS_PER_INCH);

            newBackRightTarget =
                    BackRight.getCurrentPosition() +
                    (int)(BackrightInches * COUNTS_PER_INCH);
            newExtendTarget =
                    Extend.getCurrentPosition() +
                    (int)(ExtendInches * COUNTS_PER_INCH);

            newArmTarget =
                    Arm.getCurrentPosition() +
                    (int)(ArmInches * COUNTS_PER_INCH);


            FrontLeft.setTargetPosition(newFrontLeftTarget);
            FrontRight.setTargetPosition(newFrontRightTarget);
            BackLeft.setTargetPosition(newBackLeftTarget);
            BackRight.setTargetPosition(newBackRightTarget);
            Extend.setTargetPosition(newExtendTarget);
            Arm.setTargetPosition(newArmTarget);

            // Fixed
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            FrontLeft.setPower(Math.abs(DriveSpeed));
            FrontRight.setPower(Math.abs(DriveSpeed));
            BackLeft.setPower(Math.abs(DriveSpeed));
            BackRight.setPower(Math.abs(DriveSpeed));
            Extend.setPower(Math.abs(ExtendSpeed));
            Arm.setPower(Math.abs(ArmSpeed));

            while (opModeIsActive()
                    && runtime.seconds() < timeoutS
                    && (FrontLeft.isBusy()
                    || FrontRight.isBusy()
                    || BackLeft.isBusy()
                    || BackRight.isBusy()
                    ||Extend.isBusy()
                    || Arm.isBusy())) {

                telemetry.addData("FL", FrontLeft.getCurrentPosition());
                telemetry.addData("FR", FrontRight.getCurrentPosition());
                telemetry.addData("BL", BackLeft.getCurrentPosition());
                telemetry.addData("BR", BackRight.getCurrentPosition());
                telemetry.update();
            }

            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
            Extend.setPower(0);
            Arm.setPower(0);

            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(INPUT_DELAY);
        }
    }
    public void rawPowerDrive(double FrontLeftSpeed, double FrontRightSpeed, double BackRightSpeed, 
    double BackLeftSpeed, double ExtendSpeed, double ArmSpeed) {
        FrontLeft.setPower(FrontLeftSpeed);
        FrontRight.setPower(FrontRightSpeed);
        BackRight.setPower(BackRightSpeed);
        BackLeft.setPower(BackLeftSpeed);
        Extend.setPower(ExtendSpeed);
        Arm.setPower(ArmSpeed);
    }
    public void powerDrive(double FrontLeftSpeed, double FrontRightSpeed, double BackRightSpeed,
    double BackLeftSpeed) {
        rawPowerDrive(FrontLeftSpeed, FrontRightSpeed, BackRightSpeed, BackLeftSpeed,0,0);
    }
    public void turnToHeading(double maxTurnSpeed, double heading) {
        getSteeringCorrection(heading, 0.02);
        while (opModeIsActive() && (Math.abs(headingError) > 1)) {
            double turnSpeed = getSteeringCorrection(heading, 0.03);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            turnSpeed *= -1;
            powerDrive(turnSpeed, -1 * turnSpeed, -1 * turnSpeed, turnSpeed);
        }
        powerDrive(0, 0, 0, 0);
    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double targetHeading = desiredHeading;
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        headingError = targetHeading - heading;
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;
        return Range.clip(headingError * proportionalGain, -1, 1);
    }
}
    
    
    
    
