package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrive;

@Autonomous(name="LeftAuto", group="Robot")
public final class LeftAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Robot robot = new Robot(this,true);

        waitForStart();

        /* -------------------------------------------------------------------------------------- */

        beginPose = new Pose2d(0, 0, Math.toRadians(0.00));
        drive.setPose(beginPose);
        Actions.runBlocking(drive.actionBuilder(beginPose)
                .afterTime(0, robot.arm.autoGoToHigh())
                .strafeToLinearHeading(new Vector2d(11, -20), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(28, -4), Math.toRadians(43))
                .afterTime(0, robot.arm.intakeOut())
                .waitSeconds(2)
                .stopAndAdd(robot.arm.intakeIn())
                .afterTime(1, robot.arm.autoGoToIntake1())
                .strafeToLinearHeading(new Vector2d(0, -25), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(0, -39), Math.toRadians(0))
                .stopAndAdd(robot.arm.autoGoToIntake2())
                .strafeToLinearHeading(new Vector2d(7.5, -39), Math.toRadians(0))
                .waitSeconds(1)
                .afterTime(0, robot.arm.autoGoToHigh())
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(28, -4), Math.toRadians(43))
                .stopAndAdd(robot.arm.intakeOut())
                .waitSeconds(2)
                .stopAndAdd(robot.arm.intakeIn())
                .afterTime(1, robot.arm.autoGoToIntake2())
                .strafeToLinearHeading(new Vector2d(5, -25), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(5, -40), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(18, -40), Math.toRadians(0))
                .waitSeconds(1)
                .afterTime(0, robot.arm.autoGoToHigh())
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(28, -4), Math.toRadians(43))
                .stopAndAdd(robot.arm.intakeOut())
                .waitSeconds(2)
                .stopAndAdd(robot.arm.autoGoToHome())
                .build());
    }
}