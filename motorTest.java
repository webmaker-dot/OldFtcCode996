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

@Autonomous(name="motorTest", group="Robot")
public class motorTest extends LinearOpMode {

    //Motors
    private DcMotor FrontRight = null;
    private DcMotor BackRight = null;
    private DcMotor BackLeft = null;
    private DcMotor FrontLeft = null;
    
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
    static final double DEFAULT_TIMEOUT = 5;
    double headingError;
    static final long INPUT_DELAY = 0;
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
        
        //Set motor direction
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        
        // Reset drive encoders
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        
        imu.resetYaw();
        encoderDrive(DRIVE_SPEED, 10,10,10,10, DEFAULT_TIMEOUT);
        telemetry.addData("Path", "Complete");
        telemetry.update();

        sleep(1000);
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
    public void powerDrive(double frontLeftPower, double frontRightPower, double backRightPower, double backLeftPower) {
        FrontLeft.setPower(frontLeftPower);
        FrontRight.setPower(frontRightPower);
        BackRight.setPower(backRightPower);
        BackLeft.setPower(backLeftPower);
    }

}


