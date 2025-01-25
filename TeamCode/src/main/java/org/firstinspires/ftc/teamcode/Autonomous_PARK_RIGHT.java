package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous_V1_STABLE", group="Linear OpMode")
@Config
public final class Autonomous_PARK_RIGHT extends LinearOpMode {
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

        Pose2d beginPose = new Pose2d(-12.66, 67.75, Math.toRadians(-90));
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
        ArmLift.setTargetPosition(0);
        ArmExtend.setTargetPosition(30);
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(-54.77, 63.28), Math.toRadians(181.30))
                        .build());



}   }