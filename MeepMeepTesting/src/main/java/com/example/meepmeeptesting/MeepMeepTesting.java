package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(39.03, 62.85, Math.toRadians(-90.00))
                        Actions.runBlocking(
                                drive.actionBuilder(beginPose)
                                        .splineTo(new Vector2d(66.05, 55.69), Math.toRadians(45.00))
                                        .build());
        sleep(2000);
        new Pose2d(66.05,55.69, Math.toRadians(45));
        claw.setPosition(1);
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineToConstantHeading(new Vector2d(47.11, 42.01), Math.toRadians(45.00))
                        .waitSeconds(0)
                        .turn(Math.toRadians(45))
                        .build());




















        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}