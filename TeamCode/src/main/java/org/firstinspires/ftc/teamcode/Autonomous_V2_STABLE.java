package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous_V2_STABLE", group="Linear OpMode")
@Config
public final class Autonomous_V2_STABLE extends LinearOpMode {

    public static double x1= 46;
    public static double x2= 58;
    public static double x3= 60.00;
    public static double y1= 40.01;
    public static int E1=1400;
    public static int E2=2000;


    public static double y2= 22;
    public static double y3= 33;
    public static double by=51.69;
    public static double bx=63.05;
    public static double BT=5;
    public static double BT3=0;
    public static double side=.4;

    private DcMotor ArmLift = null;
    private TouchSensor Armbutton = null;
    private Servo claw = null;
    private Servo clawY;
    private Servo clawZ;
    private DcMotor ArmExtend = null;
    @Override
    public void runOpMode() throws InterruptedException {
        ArmLift = hardwareMap.get(DcMotor.class,"Arm lift");
        ArmExtend = hardwareMap.get(DcMotor.class,"Arm extender");
        Armbutton = hardwareMap.get(TouchSensor.class,"button");

        claw = hardwareMap.get(Servo.class,"Grip");
        clawZ=hardwareMap.get(Servo.class,"Twist");
        clawY=hardwareMap.get(Servo.class,"Wrist");
        ArmExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmExtend.setTargetPosition(0);
        ArmExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmLift.setTargetPosition(0);
        ArmLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        claw.setPosition(1);
        clawZ.setPosition(-1);
        double down = side;

        Pose2d beginPose = new Pose2d(11.19, 62.22, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        claw.setPosition(1);
        clawY.setPosition(1);
        clawZ.setPosition(-1);
        ArmLift.setPower(-1);
        while (!Armbutton.isPressed()){
            telemetry.update();
        }
        ArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmExtend.setPower(1);
        ArmLift.setPower(1);
        ArmLift.setTargetPosition(2500);

        ArmExtend.setTargetPosition(2100);
        clawY.setPosition(1);
        sleep(500);
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToConstantHeading(new Vector2d(11.19,50), Math.toRadians(270))
                        .build());
        Pose2d twoPose = drive.pose;
        ArmLift.setTargetPosition(1900);
        sleep(500);
        claw.setPosition(0);
        sleep(500);
        ArmExtend.setTargetPosition(300);
        ArmLift.setTargetPosition(0);
        clawY.setPosition(down);
        clawZ.setPosition(1);

        Actions.runBlocking(
                drive.actionBuilder(twoPose)
                        //.waitSeconds(.5)
                        .splineToLinearHeading(new Pose2d(34.78, 46.69, Math.toRadians(0.00)), Math.toRadians(0.00))
                        .splineToLinearHeading(new Pose2d(x1, y2, Math.toRadians(BT)), Math.toRadians(BT))
                        .build());

        ///befor pick up
        claw.setPosition(1);
        sleep(500);
        ArmLift.setTargetPosition(4000);
        sleep(500);
        ArmExtend.setTargetPosition(3200);
        clawY.setPosition(1);
        clawZ.setPosition(-1);

        Pose2d place2 = drive.pose;
        Actions.runBlocking(
                drive.actionBuilder(place2)
                        .splineToConstantHeading(new Vector2d(x1,y1), Math.toRadians(45))
                        .splineToLinearHeading(new Pose2d(bx, by, Math.toRadians(45)), Math.toRadians(45))
                        .build());

        sleep(500);
        claw.setPosition(0);

        //round 2
       Pose2d move1 = drive.pose;
        Actions.runBlocking(
                drive.actionBuilder(move1)

                        .splineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(BT)), Math.toRadians(BT))
                        .build());

        ArmExtend.setTargetPosition(300);
        ArmLift.setTargetPosition(0);
        clawY.setPosition(down);
        clawZ.setPosition(1);
        sleep(1500);
        ///befor pick up
        claw.setPosition(1);
        sleep(500);
        ArmLift.setTargetPosition(4000);
        sleep(500);
        ArmExtend.setTargetPosition(3200);
        clawY.setPosition(1);
        clawZ.setPosition(-1);
        Pose2d place3 = drive.pose;
        Actions.runBlocking(
                drive.actionBuilder(place3)
                        .splineToConstantHeading(new Vector2d(x2,y1), Math.toRadians(45))
                        .splineToLinearHeading(new Pose2d(bx, by, Math.toRadians(45)), Math.toRadians(45))
                        .build());
        claw.setPosition(0);
        //ArmExtend.setTargetPosition(20);
        //for thrid block

        Pose2d move2 = drive.pose;
        Actions.runBlocking(
                drive.actionBuilder(move2)

                        .splineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(BT3)), Math.toRadians(BT3))
                        .build());

        ArmExtend.setTargetPosition(E1);
        ArmLift.setTargetPosition(0);
        clawZ.setPosition(1);
        ArmLift.setTargetPosition(0);
        clawY.setPosition(down);
        sleep(2000);
        ///befor pick up
        claw.setPosition(1);
        sleep(500);
        ArmLift.setTargetPosition(4000);
        sleep(500);
        ArmExtend.setTargetPosition(3200);
        clawY.setPosition(1);
        clawZ.setPosition(-1);
        Pose2d place4 = drive.pose;
        Actions.runBlocking(
                drive.actionBuilder(place4)
                        .splineToConstantHeading(new Vector2d(x1,y2), Math.toRadians(45))
                        .splineToLinearHeading(new Pose2d(bx, by, Math.toRadians(45)), Math.toRadians(45))
                        .build());
        Pose2d back2 = drive.pose;
        sleep(500);
        claw.setPosition(0);
        Actions.runBlocking(
                drive.actionBuilder(back2)
                        .splineTo(new Vector2d(49.81, 42.42), Math.toRadians(215.18))
                        .splineTo(new Vector2d(41.11, 11.31), Math.toRadians(221.01))
                        //.splineToSplineHeading(new Pose2d(30.00, 8.00, Math.toRadians(180.00)), Math.toRadians(180.00))
                        .build()


        );
        ArmLift.setTargetPosition(2900);
        ArmExtend.setTargetPosition(1000);
        while (opModeIsActive()){
            telemetry.update();
        }
    }
}