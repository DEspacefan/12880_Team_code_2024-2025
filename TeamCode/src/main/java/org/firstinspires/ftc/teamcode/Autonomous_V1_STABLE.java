package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous_V1_STABLE", group="Linear OpMode")
@Config
public final class Autonomous_V1_STABLE extends LinearOpMode {
    private TouchSensor Armbutton = null;
    private DcMotor ArmLift = null;
    private Servo claw = null;
    private DcMotor ArmExtend = null;
    @Override
    public void runOpMode() throws InterruptedException {
        ArmLift = hardwareMap.get(DcMotor.class,"Arm lift");
        ArmExtend = hardwareMap.get(DcMotor.class,"Arm extender");
        claw = hardwareMap.get(Servo.class,"claw");
        Armbutton = hardwareMap.get(TouchSensor.class,"button");
        ArmExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmExtend.setTargetPosition(0);
        ArmExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmLift.setTargetPosition(0);
        ArmLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d beginPose = new Pose2d(18.19, 62.22, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        claw.setPosition(0);
        ArmLift.setPower(-1);
        while (!Armbutton.isPressed()){
            telemetry.update();
        }
        ArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmExtend.setPower(1);
        ArmLift.setPower(1);
        ArmLift.setTargetPosition(4200);
        ArmExtend.setTargetPosition(3200);
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(65.05, 55.69), Math.toRadians(45.00))
                        .build());
        sleep(500);
        Pose2d twoPose = drive.pose;
//        Pose2d twoPose =new Pose2d(65.05,51.69, Math.toRadians(45));
        claw.setPosition(1);
        Actions.runBlocking(
                drive.actionBuilder(twoPose)
                        .splineToConstantHeading(new Vector2d(44.11, 42.01), Math.toRadians(45.00))
                        .waitSeconds(.5)
                        .turn(Math.toRadians(-130))
                        .waitSeconds(.5)

                        //First block
                        .splineToConstantHeading(new Vector2d(40.95, 0.55), Math.toRadians(270.00))
                        .splineToConstantHeading(new Vector2d(46.90, 0.55), Math.toRadians(-12.69))
                        .splineToConstantHeading(new Vector2d(55.90, 63.28), Math.toRadians(89.99))
                        .build());
        ArmExtend.setTargetPosition(0);
        Pose2d threepose = drive.pose;
        Actions.runBlocking(
                drive.actionBuilder(threepose)
                        //Second block
                        .splineToConstantHeading(new Vector2d(55.90,0.55),Math.toRadians(270.00))

                        //Third block


                        .build());
    }
}