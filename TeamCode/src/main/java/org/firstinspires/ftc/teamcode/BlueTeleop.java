package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Blue Teleop", group="Linear OpMode")
public class BlueTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, false, false, true, true, true);

        waitForStart();

        while (opModeIsActive()) {
            robot.runRobot();
        }
    }
}
