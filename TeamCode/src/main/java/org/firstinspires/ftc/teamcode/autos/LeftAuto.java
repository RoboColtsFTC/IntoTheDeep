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

        beginPose = new Pose2d(33.06, 60.91, Math.toRadians(0.00));
        drive.setPose(beginPose);
        Actions.runBlocking(drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(41.77, 38.61), Math.toRadians(45.00))
                .stopAndAdd(robot.arm.autoGoToHigh())
                .strafeToLinearHeading(new Vector2d(61, 61), Math.toRadians(45))
                .stopAndAdd(robot.arm.intakeOn())
                .waitSeconds(3)
                .stopAndAdd(robot.arm.intakeOff())
                .stopAndAdd(robot.arm.autoGoToHome())
                .strafeToLinearHeading(new Vector2d(37, 16), Math.toRadians(270.00))
                .build());
    }
}