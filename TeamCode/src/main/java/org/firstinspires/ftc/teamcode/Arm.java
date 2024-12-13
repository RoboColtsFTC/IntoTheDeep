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

    private boolean climbMode = false;

    public Arm(LinearOpMode opMode){
        this.opMode = opMode;

        this.operator = opMode.gamepad2;

        rightExtension  = opMode.hardwareMap.get(DcMotor.class, "right_extension");
        rightExtension.setDirection(DcMotor.Direction.REVERSE);
        rightExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftExtension  = opMode.hardwareMap.get(DcMotor.class, "left_extension");
        leftExtension.setDirection(DcMotor.Direction.FORWARD);
        leftExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm  = opMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = opMode.hardwareMap.get(CRServo.class, "intake");

        if(Robot.auto){
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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

    public void pullUp(){
        if(arm.getCurrentPosition() < 50){
            positionMotor(arm, 45, 1);

            if(arm.getCurrentPosition() > 40){
                positionMotor(rightExtension, -75, 1);
                positionMotor(leftExtension, -75, 1);
            }
        } else {
            positionMotor(rightExtension, -75, 1);
            positionMotor(leftExtension, -75, 1);

            if(rightExtension.getCurrentPosition() > -2000){
                positionMotor(arm, 25, .5);
            }
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

    public Action autoGoToIntake1(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(rightExtension.getCurrentPosition() < -2000){
                    positionMotor(arm, 1770, 1);
                } else if(rightExtension.getCurrentPosition() < -200){
                    positionMotor(arm, -175, 1);
                } else {
                    positionMotor(arm, 45, 1);
                }

                positionMotor(rightExtension, -400, 1);
                positionMotor(leftExtension, -400, 1);

                return !((rightExtension.getCurrentPosition() < -390)
                        && (arm.getCurrentPosition() < -170));
            }
        };
    }

    public Action autoGoToIntake2(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(rightExtension.getCurrentPosition() < -200) {
                    if (arm.getCurrentPosition() > -170){
                        positionMotor(arm, -200, 1);
                    } else {
                       arm.setPower(0);
                    }
                } else {
                    positionMotor(arm, 45, 1);
                }

                positionMotor(rightExtension, -800, 1);
                positionMotor(leftExtension, -800, 1);

                return !((rightExtension.getCurrentPosition() < -690)
                        && (arm.getCurrentPosition() < -170));
            }
        };
    }

    public void goToHigh(){
        positionMotor(arm, 1770, 1);

        if(arm.getCurrentPosition() > 1750){
            positionMotor(rightExtension, -3100, 1);
            positionMotor(leftExtension, -3100, 1);
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

    public Action intakeOut(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(-1);

                return false;
            }
        };
    }

    public Action intakeIn(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(1);

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
        positionMotor(arm, 1675, 1);

        if(arm.getCurrentPosition() > 1640){
            positionMotor(rightExtension, -1300, 1);
            positionMotor(leftExtension, -1300, 1);
        }
    }

    public void goToClimb(){
        positionMotor(arm, 2000, 1);
    }

    public void raiseExtensionClimb(){
        positionMotor(rightExtension, -3000, 1);
        positionMotor(leftExtension, -3000, 1);
    }

    public void turnOffArm(){
        rightExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightExtension.setPower(0);

        leftExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftExtension.setPower(0);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(0);
    }

    public void furtherIntake(){
        if(rightExtension.getCurrentPosition() < -200){
            positionMotor(arm, -110, .5);
        } else {
            positionMotor(arm, 45, 1);
        }

        positionMotor(rightExtension, -2100, 1);
        positionMotor(leftExtension, -2100, 1);
    }

    public void run() {
        if(operator.left_trigger > .5){
            intake.setPower(1);
        } else if (operator.right_trigger > .5){
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        if(operator.back){
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
// 2100
//110
        if(!climbMode){
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
                raiseExtensionClimb();
            } else if(operator.dpad_down){
                climbMode = true;
            } else if(Math.abs(operator.left_stick_x) > .5 || Math.abs(operator.left_stick_y) > .5){
              furtherIntake();
            } else {
                turnOffArm();
            }
        } else {
            if(operator.back){
                climbMode = false;
            }

            pullUp();
        }

        if(operator.right_bumper){
            arm.setPower(-.25);
            rightExtension.setPower(-.25);
            leftExtension.setPower(-.25);
        }

        if(operator.left_bumper){
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



        opMode.telemetry.addData("Right Extension: ", rightExtension.getCurrentPosition());
        opMode.telemetry.addData("Left Extension: ", leftExtension.getCurrentPosition());
        opMode.telemetry.addData("Arm: ", arm.getCurrentPosition());
    }
}