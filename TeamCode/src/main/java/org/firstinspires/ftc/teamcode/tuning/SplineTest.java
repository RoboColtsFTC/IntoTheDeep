package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        /* -------------------------------------------------------------------------------------- */


        beginPose = new Pose2d(-36.04, -68.65, Math.toRadians(128.66));
        drive.setPose(beginPose);
        Actions.runBlocking(drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-49.14, -29.05), Math.toRadians(90.00))
                .strafeToLinearHeading(new Vector2d(-60.50, -65.16), Math.toRadians(252.54))
                .strafeToLinearHeading(new Vector2d(-60.06, -27.74), Math.toRadians(89.33))
                .strafeToLinearHeading(new Vector2d(-63.41, -65.16), Math.toRadians(264.89))
                .strafeToLinearHeading(new Vector2d(-69.38, -28.47), Math.toRadians(99.24))
                .build());

    }
}