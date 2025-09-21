/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.DriveUtil2025;
import org.firstinspires.ftc.teamcode.utilities.IntakeUtil;


@TeleOp(name="Basic: Linear OpMode2", group="Linear OpMode")

public class BasicOpMode_Linear extends LinearOpMode {

    private static final double DRIVE_SPEED = .75;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DriveUtil2025 drive;
    private IntakeUtil intake;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize hardware here

        drive = new DriveUtil2025(this);
        initializeHardware();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            doDriveControls();
            doIntakeControls();
            doTelemetry();

        }
    }

    private void doTelemetry() {
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

    }

    private void doIntakeControls() {
        if(gamepad1.right_bumper){
            intake.setIntakeMotorPower(-1);

        } else if (gamepad1.left_bumper){
            intake.setIntakeMotorPower(1);
        } else {
            intake.setIntakeMotorPower(0);

        }
    }

    private void doDriveControls() {
        // Get gamepad inputs
//            double driveInput = -gamepad1.left_stick_y; // Forward/Backward (often inverted)
//            double strafeInput = gamepad1.left_stick_x; // Strafe Left/Right
//            double turnInput = gamepad1.right_stick_x;  // Turn Left/Right
        double rawLeftStickY = gamepad1.left_stick_y; // Forward/Backward
        double rawLeftStickX = gamepad1.left_stick_x;  // Strafe
        double rawRightStickX = gamepad1.right_stick_x; // Turn
        // Apply deadzone first (optional but recommended)
        double deadzone = 0.1;
        rawLeftStickY = Math.abs(rawLeftStickY) < deadzone ? 0 : rawLeftStickY;
        rawLeftStickX = Math.abs(rawLeftStickX) < deadzone ? 0 : rawLeftStickX;
        rawRightStickX = Math.abs(rawRightStickX) < deadzone ? 0 : rawRightStickX;

// Apply scaling (e.g., squaring)
        double scaledDriveInput = Math.copySign(rawLeftStickY * rawLeftStickY, rawLeftStickY);
        double scaledStrafeInput = Math.copySign(rawLeftStickX * rawLeftStickX, rawLeftStickX);
        double scaledTurnInput = Math.copySign(rawRightStickX * rawRightStickX, rawRightStickX);

// Use scaled inputs for driving
        drive.arcadeDrive(scaledStrafeInput, scaledDriveInput, scaledTurnInput, gamepad1.right_stick_y, DRIVE_SPEED);

        //drive.arcadeDrive(strafeInput, driveInput, turnInput, gamepad1.right_stick_y, DRIVE_SPEED);

    }

    private void initializeHardware() throws InterruptedException {
        Thread.sleep(250); //give enough time to initialize and set light colors
        drive.init(hardwareMap,telemetry); //initialize the drive subsystem
        intake = new IntakeUtil(hardwareMap);
    }
}
