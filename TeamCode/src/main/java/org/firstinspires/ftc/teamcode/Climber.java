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

    private boolean climbUp = false;
    private boolean lastClimbUp = false;
    private boolean climbDown = false;
    private boolean lastClimbDown = false;

    private boolean upClimbing = false;
    private boolean downClimbing = false;

    public Climber(LinearOpMode opMode){
        this.opMode = opMode;
        this.operator = opMode.gamepad2;

        leftJack = opMode.hardwareMap.get(CRServo.class, "left_jack");
        rightJack = opMode.hardwareMap.get(CRServo.class, "right_jack");

    }

    public void run() {

        if(operator.dpad_right){
            climbUp = true;
        } else if(operator.dpad_left){
            climbDown = true;
        }

        if(lastClimbUp != climbUp && climbUp){
            opMode.resetRuntime();

            upClimbing = true;
        }

        if(lastClimbDown != climbDown && climbDown){
            opMode.resetRuntime();

            downClimbing = true;
        }

        if(upClimbing){
            leftJack.setPower(1);
            rightJack.setPower(1);

            if(opMode.getRuntime() > 8.4){
                upClimbing = false;
            }
        } else if(downClimbing){
            rightJack.setPower(-1);
            leftJack.setPower(-1);

            if(opMode.getRuntime() > 8.4){
                downClimbing = false;
            }
        } else {
            rightJack.setPower(0);
            leftJack.setPower(0);
        }



/*        if (operator.right_bumper) {
            leftJack.setPower(1);
            rightJack.setPower(1);
        } else if (operator.left_bumper) {
            leftJack.setPower(-1);
            rightJack.setPower(-1);
        } else {
            rightJack.setPower(0);
            leftJack.setPower(0);
        }*/

        lastClimbDown = climbDown;
        lastClimbUp = climbUp;
    }
}
