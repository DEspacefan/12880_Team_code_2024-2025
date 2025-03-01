/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import android.util.Size;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;


import java.util.List;


/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="2024-2025 Code TeleOp", group="Linear OpMode")

public class Teleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor ArmLift = null;
    private DcMotor ArmExtend = null;
    private Servo claw = null;
    private TouchSensor Armbutton = null;
    private Servo clawZ;
    private Servo clawY;




    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FL/LO");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "RL/RO");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RR/BO");
        ArmLift = hardwareMap.get(DcMotor.class,"Arm lift"); // Arm Angle 0 thru -2000?
        ArmExtend = hardwareMap.get(DcMotor.class,"Arm extender"); // Arm Extends
        claw = hardwareMap.get(Servo.class,"Grip");
        Armbutton = hardwareMap.get(TouchSensor.class,"button");
        clawZ=hardwareMap.get(Servo.class,"Twist");
        clawY=hardwareMap.get(Servo.class,"Wrist");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        ArmExtend.setDirection(DcMotorSimple.Direction.FORWARD);
        ArmLift.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmLift.setTargetPosition(0);
        ArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int armLimit;
        int Armtarget =0;
        boolean armoversize = false;





        waitForStart();
        runtime.reset();
        clawZ.setPosition(-1);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            double power = 0;
            double button = 0;




            int currentArmLiftPosition = ArmLift.getCurrentPosition();
            int currentArmExtend = ArmExtend.getCurrentPosition();

            //arm power code
            if (gamepad1.left_bumper) {
                power = .50;
            } else if (gamepad1.right_bumper) {
                power = .25;
            } else power = 1;


            // Use this variable to apply limiter
            // If 0 -> -2300, ~> 45

                    //maxLegth(currentArmLiftPosition); //TODO: Replace 0 with current robot angle of arm variable

            //
            //Arm extend code
            if (ArmLift.getCurrentPosition()<1000){
                armLimit=2300;
            }else armLimit=3200;

            //TODO: Currently just sets Arm target without checking max reach
            if (ArmExtend.getCurrentPosition()>=armLimit&& gamepad2.right_stick_y<0) {
                ArmExtend.setPower(0);
            } else if (ArmExtend.getCurrentPosition()>=armLimit+100) {
                ArmExtend.setPower(-1);
            } else if (ArmExtend.getCurrentPosition()<=0 && gamepad2.right_stick_y>0) {
                ArmExtend.setPower(0);
            }else ArmExtend.setPower(-gamepad2.right_stick_y);



            //arm button lift code
            if (Armbutton.isPressed()) {
                button = 1;
            } else button = 0;

            //TODO: Name the function
            ArmLift.setPower(1); //Set power to arm
             if (gamepad2.a){
                ArmLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ArmLift.setPower(1);
                Armtarget = 0;
            } else if (gamepad2.left_stick_y <=-.2) {
                Armtarget = Armtarget + 50;
            } else if (gamepad2.left_stick_y >=.2 && Armtarget>100) {
                Armtarget = Armtarget - 50;
            } else if (Armtarget >= 5000) {
                Armtarget = 5000;
            } else if (gamepad2.x) {
                Armtarget=2000;
            }
            if (button == 1){
                ArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            ArmLift.setTargetPosition(Armtarget);


            //claw code
            //closed
            if (gamepad2.left_bumper){
                claw.setPosition(0);
            }
            //open
            if (gamepad2.right_bumper) {
                claw.setPosition(1);
            }
            if (gamepad2.dpad_down){
                clawY.setPosition(.4);
            }
            if (gamepad2.dpad_up){
                clawY.setPosition(1);
            }
            if (gamepad2.dpad_right){
                clawZ.setPosition(1);
            }
            if (gamepad2.dpad_left){
                clawZ.setPosition(-1);
            }

            if (gamepad2.y){
                claw.setPosition(.3);
            }


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = (axial + lateral + yaw) * power;
            double rightFrontPower = (axial - lateral - yaw) * power;
            double leftBackPower   = (axial - lateral + yaw) * power;
            double rightBackPower  = (axial + lateral - yaw) * power;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("arm posision",ArmLift.getCurrentPosition());
            telemetry.addData("arm extend",ArmExtend.getCurrentPosition());
            telemetry.addData("armpower",gamepad2.right_stick_y);
            telemetry.addData("button",button);

            telemetry.update();
        }

    }
    // Set a invisible boundary to prevent the arm from leaving the allowable space
    public int maxLegth(int currentAngle) {

        if(currentAngle == 0)
            return -2300;
        return(-5*currentAngle-2300);
    }
}