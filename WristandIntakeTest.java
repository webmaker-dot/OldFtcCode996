package com.deltarobotics27115.membercode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class WristandIntakeTest extends LinearOpMode {
    public Servo wrist;
    public Servo intake;

    @Override
    public void runOpMode() {
        wrist = hardwareMap.get(Servo.class, "Wrist");
        intake = hardwareMap.get(Servo.class, "Intake");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();
        double wristcurrentposition = 0.5;
        while (opModeIsActive()) {

            // Wrist movement
            if (gamepad2.left_trigger > 0.2) {
                while (gamepad2.left_trigger > 0.2) {
                    wristcurrentposition = wristcurrentposition + 0.05;
                    telemetry.addData("Wrist current position", wristcurrentposition);
                    //sleep(250);
                }
            } if (gamepad2.right_trigger > 0.2) {
                while (gamepad2.right_trigger > 0.2) {
                    wristcurrentposition = wristcurrentposition - 0.05;
                    telemetry.addData("Wrist current position", wristcurrentposition);
                    //sleep(250);
                }
            }
            
            wrist.setPosition(wristcurrentposition);

            // Intake
            if (gamepad2.left_bumper) {
                intake.setPosition(1); // Move intake forward
                telemetry.addData("Intake Status", "Moving Forward");
            } else if (gamepad2.right_bumper) {
                intake.setPosition(-1); // Move intake backward
                telemetry.addData("Intake Status", "Moving Backward");
            }

            // Display telemetry data
            telemetry.update();
        }
    }
}

