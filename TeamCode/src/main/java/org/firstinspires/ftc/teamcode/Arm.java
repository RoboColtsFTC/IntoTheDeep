package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {

    private LinearOpMode opMode;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightExtension = null;
    private DcMotor leftExtension = null;
    private DcMotor arm = null;

    private Gamepad operator;

    private CRServo intake;

    public Arm(LinearOpMode opMode){
        this.opMode = opMode;

        this.operator = opMode.gamepad2;

        rightExtension  = opMode.hardwareMap.get(DcMotor.class, "right_extension");
        rightExtension.setDirection(DcMotor.Direction.REVERSE);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftExtension  = opMode.hardwareMap.get(DcMotor.class, "left_extension");
        leftExtension.setDirection(DcMotor.Direction.FORWARD);
        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm  = opMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = opMode.hardwareMap.get(CRServo.class, "intake");
    }

    public void positionMotor(DcMotor motor, int target, double speed){
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(Math.abs(target - motor.getCurrentPosition()) > 400){
            motor.setPower(speed);
        } else {
            motor.setPower(speed / 2);
        }
    }

    public void goToHome(){
        if(arm.getCurrentPosition() < 50){
            positionMotor(arm, 45, 1);

            if(arm.getCurrentPosition() > 40){
                positionMotor(rightExtension, -75, 1);
                positionMotor(leftExtension, -75, 1);
            }
        } else {
            positionMotor(rightExtension, -75, 1);
            positionMotor(leftExtension, -75, 1);

            if(rightExtension.getCurrentPosition() > -250){
                positionMotor(arm, 25, .5);
            }
        }
    }

    public Action autoGoToHome(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                goToHome();

                return !((rightExtension.getCurrentPosition() > -100)
                        && (arm.getCurrentPosition() < 50));
            }
        };
    }

    public void goToIntake(){
        if(rightExtension.getCurrentPosition() < -200){
            positionMotor(arm, -115, .5);
        } else {
            positionMotor(arm, 45, 1);
        }

        positionMotor(rightExtension, -1500, 1);
        positionMotor(leftExtension, -1500, 1);
    }

    public void goToHigh(){
        positionMotor(arm, 1770, 1);

        if(arm.getCurrentPosition() > 1750){
            positionMotor(rightExtension, -3050, 1);
            positionMotor(leftExtension, -3050, 1);
        }
    }

    public Action autoGoToHigh(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                goToHigh();

                return !(rightExtension.getCurrentPosition() < -3025);
            }
        };
    }

    public Action intakeOn(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(-1);

                return false;
            }
        };
    }

    public Action intakeOff(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(0);

                return false;
            }
        };
    }

    public void goToMid(){
        positionMotor(arm, 1660, 1);

        if(arm.getCurrentPosition() > 1640){
            positionMotor(rightExtension, -1400, 1);
            positionMotor(leftExtension, -1400, 1);
        }
    }

    public void goToClimb(){
        positionMotor(arm, 2000, 1);
    }

    public void extendClimber(){
        positionMotor(rightExtension, -2950, 1);
        positionMotor(leftExtension, -2950, 1);
    }

    public void turnOffArm(){
        rightExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightExtension.setPower(0);

        leftExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftExtension.setPower(0);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(0);
    }

    public void run() {
        if(operator.left_trigger > .5){
            intake.setPower(1);
        } else if (operator.right_trigger > .5){
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        if(operator.a) { // Home
            goToHome();
        } else if(operator.y) { // High
            goToHigh();
        } else if(operator.b) { // Mid
            goToMid();
        } else if(operator.x) {
            goToIntake();
        } else if(operator.dpad_right) {
            goToClimb();
        } else if(operator.dpad_up) {
            rightExtension.setPower(-.25);
            leftExtension.setPower(-.25);
        } else if (operator.dpad_down){
            rightExtension.setPower(.25);
            leftExtension.setPower(.25);
        } else {
            turnOffArm();
        }


        opMode.telemetry.addData("Right Extension: ", rightExtension.getCurrentPosition());
        opMode.telemetry.addData("Left Extension: ", leftExtension.getCurrentPosition());
        opMode.telemetry.addData("Arm: ", arm.getCurrentPosition());
    }
}