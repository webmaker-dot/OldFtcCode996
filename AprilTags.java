package org.firstinspires.ftc.robotcontroller.external.samples;

import android.util.Size;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;


import java.util.List;
@TeleOp(name = "AprilTagTest")
public class AprilTags extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private ExposureControl exposureControl;
    private GainControl gainControl;
    
    private double[] relativeXYZ = {0,0,0};
    private double[] fieldXY = {0,0};
    private double pitch = 45;
    private double sinPitch = Math.sin(pitch * 2 * Math.PI / 360.0);
    
    
    //Gyro
    private BHI260IMU imu;
    
    private double ballTerminalVelocity = 580; //inches / second, someone find the actual value
    private double gravity = 385; //inches  / second ^ 2
    private double extraHeight = 5; //inches
    @Override
    public void runOpMode() {

        
        initImu();
        imu.resetYaw();
        initAprilTag();
        
        waitForStart();
        imu.resetYaw();
        try {
            exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl != null) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(2, TimeUnit.MILLISECONDS);
            }
            if (gainControl != null) {
                gainControl.setGain(255);
            }
        } catch (Exception e) {
            telemetry.addData("Camera Control Error", e.getMessage());
        }

        while (opModeIsActive()) {
            telemetryAprilTag();
            

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            /*if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }*/

            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        //visionPortal.close();

    } 
    private void initImu() {
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
                        //TODO: Once the robot exists change
        imu.initialize(parameters);
        
    }
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            //.setDrawTagOutline(true)
            //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
            //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

            // == CAMERA CALIBRATION ==
            // If you do not manually specify calibration parameters, the SDK will attempt
            // to load a predefined calibration for your camera.
            //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
            // ... these parameters are fx, fy, cx, cy.
            //Set tag targets
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
            //Set camera position
            .setCameraPose(
                new Position(DistanceUnit.INCH, 0, 0,0,0),
                new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0)
            )
            .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);
        
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);
        //builder.setLiveViewContainerId(1);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                
                String clusterName = (cluster.metadata != null) ? cluster.metadata.name : "Unknown Cluster";
                telemetry.addLine(String.format("\n==== [CLUSTER] %s", clusterName));
                telemetry.addLine(String.format("Percent Found: %d%%", cluster.percentClusterFound));
                
                if (cluster.ftcPose != null) {
                    telemetry.addLine(String.format("  Rel X: %.1f, Y: %.1f, Z: %.1f in", cluster.ftcPose.x, cluster.ftcPose.y, cluster.ftcPose.z));
                    telemetry.addLine(String.format("  Range: %.1f, Bearing: %.1f, Elevation: %.1f deg", cluster.ftcPose.range, cluster.ftcPose.bearing, cluster.ftcPose.elevation));
                    if (cluster.ftcPose.y > 45) { //Change calculation if camera does not point straight up and isn't on the ground
                        telemetry.addLine("Ready to Shoot.");
                    } else {
                        telemetry.addLine("Do not Shoot.");
                    }
                }
                
                if (cluster.robotPose != null) {
                    double rx = cluster.robotPose.getPosition().x;
                    double ry = cluster.robotPose.getPosition().y;
                    telemetry.addLine(String.format("  Robot Field X,Y: %.1f, %.1f in", rx, ry));
                }
                
                if (cluster.robotPose != null && cluster.ftcPose != null) {
                    double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    double elevation = cluster.ftcPose.elevation;
                    elevation += pitch;
                    double distance = cluster.ftcPose.range;
                    //telemetry.addLine(String.format("New elevation: %.1f, Distance: %.1f",elevation,distance));
                    double height = distance * Math.sin(elevation * 2 * Math.PI / 360);
                    double floorDistance = distance * Math.cos(elevation * 2 * Math.PI / 360);
                    double verticalVelocity = ballTerminalVelocity * Math.sqrt(Math.exp((height + extraHeight) * 2 * gravity / ballTerminalVelocity / ballTerminalVelocity) - 1);
                    double horizontalVelocity = ballTerminalVelocity * (Math.exp(floorDistance * gravity / ballTerminalVelocity / ballTerminalVelocity) - 1) / Math.atan(verticalVelocity / ballTerminalVelocity);
                    telemetry.addLine(String.format("Height: %.1f, Floor Distance: %.1f",height,floorDistance));
                    telemetry.addLine(height > 45 ? "Ready to shoot." : "Dont shoot.");
                    telemetry.addLine(String.format("Predicted Vertical Velocity: %.1f in/s",verticalVelocity));
                    telemetry.addLine(String.format("Predicted Horizontal Velocity: %.1f in/s",horizontalVelocity));
                    //telemetry.addLine("" + Math.exp((height + extraHeight) * 2 * 385 / ballTerminalVelocity / ballTerminalVelocity));
                    
                }
            } 
            // 2. Otherwise, check if it's a standalone single tag
            else if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                
                String tagName = (single.metadata != null) ? single.metadata.name : "Unknown Tag";
                telemetry.addLine(String.format("\n==== (ID %d) %s", single.id, tagName));
                
                if (single.ftcPose != null) {
                    telemetry.addLine(String.format("  Rel X: %.1f, Y: %.1f, Z: %.1f in", single.ftcPose.x, single.ftcPose.y, single.ftcPose.z));
                }
            }
        } 

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    } 

}   // end class
