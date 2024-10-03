package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();


        beginPose = new Pose2d(-50.76, -32.01, Math.toRadians(270.00));
        drive.setPose(beginPose);
        Actions.runBlocking(drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-46.80, -50.61), Math.toRadians(-14.93))
                .strafeToLinearHeading(new Vector2d(4.91, -56.18), Math.toRadians(-12.12))
                .strafeToLinearHeading(new Vector2d(46.07, -61.45), Math.toRadians(-5.97))
                .strafeToLinearHeading(new Vector2d(62.19, -61.45), Math.toRadians(49.09))
                .strafeToLinearHeading(new Vector2d(68.04, -55.59), Math.toRadians(45.00))
                .build());

        beginPose = new Pose2d(-50.76, -32.01, Math.toRadians(270.00));
        drive.setPose(beginPose);
        Actions.runBlocking(drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-46.80, -50.61), Math.toRadians(-14.93))
                .strafeToLinearHeading(new Vector2d(4.91, -56.18), Math.toRadians(-12.12))
                .strafeToLinearHeading(new Vector2d(46.07, -61.45), Math.toRadians(-5.97))
                .strafeToLinearHeading(new Vector2d(62.19, -61.45), Math.toRadians(49.09))
                .strafeToLinearHeading(new Vector2d(68.04, -55.59), Math.toRadians(45.00))
                .build());


    }
}