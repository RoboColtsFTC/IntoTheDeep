package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Climber {
    private LinearOpMode opMode;

    private Gamepad operator;

    private CRServo leftJack;
    private CRServo rightJack;

    public Climber(LinearOpMode opMode){
        this.opMode = opMode;
        this.operator = opMode.gamepad2;

        leftJack = opMode.hardwareMap.get(CRServo.class, "left_jack");
        rightJack = opMode.hardwareMap.get(CRServo.class, "right_jack");

    }

    public void run() {
        if (operator.right_bumper) {
            leftJack.setPower(1);
            rightJack.setPower(1);
        } else if (operator.left_bumper) {
            leftJack.setPower(-1);
            rightJack.setPower(-1);
        } else {
            rightJack.setPower(0);
            leftJack.setPower(0);
        }
    }
}
