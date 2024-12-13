package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;

@Autonomous(name="LeftAutoFaster", group="Robot")
public final class LeftAutoFaster extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Robot robot = new Robot(this,true);

        waitForStart();

        beginPose = new Pose2d(0, 0, Math.toRadians(0.00));
        drive.setPose(beginPose);
        Actions.runBlocking(drive.actionBuilder(beginPose)
                .afterTime(0, robot.arm.autoGoToHigh())
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(11, -20, Math.toRadians(43)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(30, -3.5, Math.toRadians(43)), Math.toRadians(43))
                .stopAndAdd(robot.arm.intakeOut())
                .waitSeconds(1.25)
                .stopAndAdd(robot.arm.intakeIn())
                .afterTime(0, robot.arm.autoGoToIntake1())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(2.5, -25, Math.toRadians(0)), Math.toRadians(250))
                .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(0)), Math.toRadians(250))
                .afterTime(0, robot.arm.autoGoToIntake2())
                .strafeToLinearHeading(new Vector2d(5, -40), Math.toRadians(0))
                .afterTime(0, robot.arm.autoGoToHigh())
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(28, -4), Math.toRadians(43))
                .stopAndAdd(robot.arm.intakeOut())
                .waitSeconds(1)
                .stopAndAdd(robot.arm.intakeIn())
                .afterTime(.5, robot.arm.autoGoToIntake2())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(5, -25, Math.toRadians(0)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(5, -41, Math.toRadians(0)), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(15, -41), Math.toRadians(0))
                .afterTime(0, robot.arm.autoGoToHigh())
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(28, -4), Math.toRadians(43))
                .stopAndAdd(robot.arm.intakeOut())
                .waitSeconds(1)
                .stopAndAdd(robot.arm.intakeIn())
                .afterTime(.5, robot.arm.autoGoToIntake2())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(15, -25, Math.toRadians(0)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(15, -42, Math.toRadians(0)), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(22, -42), Math.toRadians(0), new TranslationalVelConstraint(25))
                .afterTime(0, robot.arm.autoGoToHigh())
                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(28, -4), Math.toRadians(43))
                .setTangent(135)
                .splineToSplineHeading(new Pose2d(11, -30, Math.toRadians(43)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(30, -3.5, Math.toRadians(43)), Math.toRadians(45))
                .stopAndAdd(robot.arm.intakeOut())
                .waitSeconds(1)
                .afterTime(0, robot.arm.intakeOff())
                .stopAndAdd(robot.arm.autoGoToHome())
                .build());
    }
}