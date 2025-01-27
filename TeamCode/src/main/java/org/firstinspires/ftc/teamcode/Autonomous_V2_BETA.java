package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous_V2_BETA", group="Linear OpMode")
@Config
public final class Autonomous_V2_BETA extends LinearOpMode {

    public static double x1= 55.90;
    public static double x2= 70.90;
    public static double y1= 40.01;


    public static double y2= 37;

    private DcMotor ArmLift = null;
    private TouchSensor Armbutton = null;
    private Servo claw = null;
    private DcMotor ArmExtend = null;
    @Override
    public void runOpMode() throws InterruptedException {
        ArmLift = hardwareMap.get(DcMotor.class,"Arm lift");
        ArmExtend = hardwareMap.get(DcMotor.class,"Arm extender");
        Armbutton = hardwareMap.get(TouchSensor.class,"button");
        claw = hardwareMap.get(Servo.class,"claw");
        ArmExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmExtend.setTargetPosition(0);
        ArmExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmLift.setTargetPosition(0);
        ArmLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        claw.setPosition(0);

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
        ArmLift.setTargetPosition(4500);
        ArmExtend.setTargetPosition(3200);
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(67.05, 55.69), Math.toRadians(45.00))
                        .build());
        Pose2d twoPose = drive.pose;
//        Pose2d twoPose =new Pose2d(65.05,51.69, Math.toRadians(45));
        claw.setPosition(1);
        Actions.runBlocking(
                drive.actionBuilder(twoPose)
                        .waitSeconds(.5)
                        .splineToConstantHeading(new Vector2d(x1, y1), Math.toRadians(270.00))
                        .turn(Math.toRadians(-130))
                        .build());
        ArmExtend.setTargetPosition(300);
        ArmLift.setTargetPosition(0);
        Pose2d pick1 = drive.pose;
        Actions.runBlocking(
                drive.actionBuilder(pick1)
                        //Second block
                        .waitSeconds(1)
                        .splineToConstantHeading(new Vector2d(x1,y2),Math.toRadians(270.00))
                        .build());
        claw.setPosition(0);
        sleep(500);
        ArmLift.setTargetPosition(4500);
        sleep(500);
        ArmExtend.setTargetPosition(3200);
        Pose2d place2 = drive.pose;
        Actions.runBlocking(
                drive.actionBuilder(place2)
                        .splineToConstantHeading(new Vector2d(x1,y1),Math.toRadians(270))
                        .turn(Math.toRadians(130))
                        .splineToConstantHeading(new Vector2d(65.5,55.69),Math.toRadians(45))
                        .build());

        claw.setPosition(1);
        //round 2
       Pose2d move1 = drive.pose;
        Actions.runBlocking(
                drive.actionBuilder(move1)
                        .waitSeconds(.5)
                        .splineToConstantHeading(new Vector2d(x2, y1), Math.toRadians(270.00))
                        .turn(Math.toRadians(-130))
                        .build());
        ArmExtend.setTargetPosition(300);
        ArmLift.setTargetPosition(0);
        Pose2d pick2 = drive.pose;
        Actions.runBlocking(
                drive.actionBuilder(pick2)
                        //Second block
                        .waitSeconds(1)
                        .splineToConstantHeading(new Vector2d(x2,y2),Math.toRadians(270.00))
                        .build());
        claw.setPosition(0);
        sleep(500);
        ArmLift.setTargetPosition(4500);
        sleep(500);
        ArmExtend.setTargetPosition(3200);
        Pose2d place3 = drive.pose;
        Actions.runBlocking(
                drive.actionBuilder(place3)
                        .splineToConstantHeading(new Vector2d(x2,y1),Math.toRadians(270))
                        .turn(Math.toRadians(130))
                        .splineToConstantHeading(new Vector2d(65.5,55.69),Math.toRadians(45))
                        .build());
        claw.setPosition(1);
        Pose2d back2 = drive.pose;
        Actions.runBlocking(
                drive.actionBuilder(back2)
                        .splineToConstantHeading(new Vector2d(55.5,55.69),Math.toRadians(45))
                        .build()
        );
        ArmExtend.setTargetPosition(20);
        sleep(1000);
        ArmLift.setTargetPosition(0);
        while (opModeIsActive()){
            telemetry.update();
        }
    }
}