package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;

public class Robot {
    LinearOpMode opMode;

    static boolean auto;
    static boolean red;
    static boolean useDrive;
    static boolean useArm;
    static boolean useClimber;

    public Drivetrain drivetrain;
    public Arm arm;
    public Climber climber;

    public Robot(LinearOpMode opMode, boolean auto, boolean red, boolean useDrive, boolean useArm, boolean useClimber){
        this.opMode = opMode;
        Robot.auto = auto;
        Robot.red = red;
        Robot.useDrive = useDrive;
        Robot.useArm = useArm;
        Robot.useClimber = useClimber;

        if(!auto && useDrive){
            drivetrain = new Drivetrain(this.opMode);
        }

        if(useArm){
            arm = new Arm(this.opMode);
        }

        if(useClimber){
            climber = new Climber(this.opMode);
        }
    }

    public Robot(LinearOpMode opMode, boolean red){
        this(opMode, true, red, false, true, false);
    }

    public void runRobot(){
        if(!auto && useDrive){
            drivetrain.run();
        }

        if(useArm){
            arm.run();
        }

        if(useClimber){
            climber.run();
            opMode.telemetry.update();
        }
    }
}
