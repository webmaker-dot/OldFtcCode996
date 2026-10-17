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

@Autonomous(name="Autonomous", group="Robot")
public class OptimizedAuto1 extends LinearOpMode {

    //Motors
    private DcMotor FrontRight = null;
    private DcMotor BackRight = null;
    private DcMotor BackLeft = null;
    private DcMotor FrontLeft = null;
    private DcMotor Extend = null;
    private DcMotor Arm = null;
    private Servo Wrist = null;
    private Servo Intake = null;
    
    // Sensors
    private BHI260IMU imu;
    
    // Default Values
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.1;
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.6;
    static final double GREEN_GIANT = 60;
    static final double DEFAULT_TIMEOUT = 5;
    double headingError;
    static final long INPUT_DELAY = 0;
    static final int OPEN_ANGLE = 140;
    static final int CLOSE_ANGLE = 210;
    static final double BUCKET_SPEED = 1;
    static final double BUCKET_DIS = 77;
    static final double BUCKET_TIMEOUT = 1.45;
    @Override
    public void runOpMode() {
        
        // Initialize IMU
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
        
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
        
        
        //Set motor direction
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        Arm.setDirection(DcMotor.Direction.FORWARD);
        Extend.setDirection(DcMotor.Direction.REVERSE);
        
        //Set motor brakes
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset drive encoders
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset arm encoders
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        
        imu.resetYaw();
        //Wrist to front
        setWrist(145.0, 0);
        //Close arm
        setIntake(CLOSE_ANGLE, 0);
        //Raise up 50
        encoderArm(0.8, 0, 50, 10);
        
        //Drive forward 11, Extend arm 15
        rawEncoder(0.8,11.0,11.0,11.0,11.0,15.0,0.0,1);
        //servo calibralition
        /*for (int i = 0;i < 360;i += 10) {
            setIntake(i,0.5);
            telemetry.addData("Current Angle", i);
            telemetry.update();
        }
        sleep(10000000);
        */
        //Pivot around front (turn left 22), extend arm 35, raise arm 30
        rawEncoder(BUCKET_SPEED,0,0,-22,22,35.0,30,2);
        //turn off arm brakes
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sleep(1000);
        //open arm
        setIntake(OPEN_ANGLE, 0);
        sleep(500);
        //turn brake on; begin pickuping #2
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //move backward 4, pull back arm 20, lower arm 55
        rawEncoder(0.5,-4,-4,-4,-4,-18,-55,3);
        //turn brake off
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //turn to -5
        turnToHeading(0.6,-5);
        sleep(1000);
        //close arm
        setIntake(CLOSE_ANGLE, 0);
        //Turn to -45
        turnToHeading(0.6,-45);
        setArmBrake(true);
        //Move forward, push arm out 20, pull arm up 60
        rawEncoder(BUCKET_SPEED,4,4,4,4,20,BUCKET_DIS,BUCKET_TIMEOUT);
        //turn off arm brakes and open arm
        setArmBrake(false);
        sleep(1000);
        setIntake(OPEN_ANGLE, 0);
        sleep(500);
        //begin picking up #3
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //move backward 4, pull back arm 20, lower arm 55
        rawEncoder(0.5,-4,-4,-4,-4,-16,-45,3);
        //turn brake off
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //turn to 6
        turnToHeading(0.8,15);
        sleep(1000);
        setIntake(CLOSE_ANGLE, 0);
        turnToHeading(0.6,-45);
        setArmBrake(true);
        //Move forward, push arm out 20, pull arm up 60
        rawEncoder(BUCKET_SPEED,4,4,4,4,15,BUCKET_DIS,BUCKET_TIMEOUT);
        //turn off arm brakes and open arm
        setArmBrake(false);
        sleep(1000);
        setIntake(OPEN_ANGLE, 0);
        sleep(500);
        //Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(100000);
        //together:
        // go forward a little bit
        
        // turn to -45
        // go back a little bit
        // arm
        // turn to 0
        // next:
        /*encoderArm(0.8, 0, 30, 40);
        setWrist(145.0, 2);
        setIntake(210.0, 1);
        encoderArm(0.8, 0, -15, 2);
        driveforward(11, 40);
        turnToHeading(TURN_SPEED, 0);
        extendforward(30, 40);
        setIntake(250.0, 1);
        extendforward(-30, 40);
        turnToHeading(0.25, 88);
        driveforward(97, DEFAULT_TIMEOUT);
        turnToHeading(TURN_SPEED, 90);
        encoderArm(0.8, 0, GREEN_GIANT, 40);
        extendforward(50, 40);
        encoderDrive(0.5, 0, 18, 15, 0, DEFAULT_TIMEOUT);
        encoderDrive(0.25, 3, 3, 3, 3, DEFAULT_TIMEOUT);
        turnToHeading(TURN_SPEED, 135);
        encoderDrive(0.25, 3, 3, 3, 3, DEFAULT_TIMEOUT);
        setIntake(210.0, 1);
        setIntake(250.0, 1);*/

        telemetry.addData("Path", "Complete");
        telemetry.update();

        sleep(1000);
    }
    public void setArmBrake(boolean on) {
        if (on) {
            Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }else{
            Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
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
    public double getHeading() {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return heading;
    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double targetHeading = desiredHeading;
        headingError = targetHeading - getHeading();
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;
        telemetry.addData("Desired heading", desiredHeading);
        telemetry.addData("Heading error", headingError);
        telemetry.update();
        return Range.clip(headingError * proportionalGain, -1, 1);
    }
    public void driveforward (double inches, double timeoutS){
        encoderDrive (DRIVE_SPEED, inches, inches, inches, inches, timeoutS);
    }
    public void extendforward (double inches, double timeoutS) {
        encoderArm (0.8, inches, 0, timeoutS);
    }
    public void turn (double degrees, double timeoutS){
        double newdistance = degrees/360.0*95.0;
        encoderDrive(TURN_SPEED, newdistance, -1 * newdistance, -1 * newdistance, newdistance, timeoutS);
    }
    public void powerDrive(double frontLeftPower, double frontRightPower, double backRightPower, double backLeftPower) {
        FrontLeft.setPower(frontLeftPower);
        FrontRight.setPower(frontRightPower);
        BackRight.setPower(backRightPower);
        BackLeft.setPower(backLeftPower);
    }
    public void encoderDrive(double speed,
                             double FrontleftInches,
                             double FrontrightInches,
                             double BackrightInches,
                             double BackleftInches,
                             double timeoutS) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

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

            FrontLeft.setTargetPosition(newFrontLeftTarget);
            FrontRight.setTargetPosition(newFrontRightTarget);
            BackLeft.setTargetPosition(newBackLeftTarget);
            BackRight.setTargetPosition(newBackRightTarget);

            // Fixed
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            FrontLeft.setPower(Math.abs(speed));
            FrontRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));
            BackRight.setPower(Math.abs(speed));

            while (opModeIsActive()
                    && runtime.seconds() < timeoutS
                    && (FrontLeft.isBusy()
                    || FrontRight.isBusy()
                    || BackLeft.isBusy()
                    || BackRight.isBusy())) {

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

            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(INPUT_DELAY);
        }
    }
    
    public void setWrist(double degrees,double timeoutS) {
        //1 = 300 deg
        //0 = 0 deg
        //0.5 = 150 deg
        double newpos = degrees / 300.0;
        Wrist.setPosition(newpos);
        sleep((int)(timeoutS * 1000.0));
        
    }
    
    public void setIntake(double degrees,double timeoutS) {
        //1 = 300 deg
        //0 = 0 deg
        //0.5 = 150 deg
        double newpos = degrees / 300.0;
        Intake.setPosition(newpos);
        sleep((int) (timeoutS * 1000.0));
        
    }
    
    public void encoderArm(double speed,
                             double ExtendInches,
                             double ArmInches,
                             double timeoutS) {

        int newExtendTarget;
        int newArmTarget;

        if (opModeIsActive()) {

            newExtendTarget =
                    Extend.getCurrentPosition() +
                    (int)(ExtendInches * COUNTS_PER_INCH);

            newArmTarget =
                    Arm.getCurrentPosition() +
                    (int)(ArmInches * COUNTS_PER_INCH);

            Extend.setTargetPosition(newExtendTarget);
            Arm.setTargetPosition(newArmTarget);
            // Fixed
            Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            Extend.setPower(Math.abs(speed));
            Arm.setPower(Math.abs(speed));

            while (opModeIsActive()
                    && runtime.seconds() < timeoutS
                    &&( Extend.isBusy()
                    || Arm.isBusy())) {

                // telemetry.addData("FL", FrontLeft.getCurrentPosition());
                // telemetry.addData("FR", FrontRight.getCurrentPosition());
                // telemetry.update();
            }

            Extend.setPower(0);
            Arm.setPower(0);

            Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(INPUT_DELAY);
        }
    }
    public void rawEncoder(double speed,
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

            FrontLeft.setPower(Math.abs(speed));
            FrontRight.setPower(Math.abs(speed));
            BackLeft.setPower(Math.abs(speed));
            BackRight.setPower(Math.abs(speed));
            Extend.setPower(Math.abs(speed));
            Arm.setPower(Math.abs(speed));

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

}


