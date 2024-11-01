package org.firstinspires.ftc.teamcode.drivetrain;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Drivetrain {
    MecanumDrive drive;

    LinearOpMode opMode;

    double width = .3429;
    double length = .3429;

    Translation2d m_frontLeftLocation = new Translation2d(width/2.0, length/2.0);
    Translation2d m_frontRightLocation = new Translation2d(width/2.0, -length/2.0);
    Translation2d m_backLeftLocation = new Translation2d(-width/2.0, length/2.0);
    Translation2d m_backRightLocation = new Translation2d(-width/2.0, -length/2.0);

    // Creating my kinematics object using the wheel locations.
    MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
                    m_frontLeftLocation, m_frontRightLocation,
                    m_backLeftLocation, m_backRightLocation);

    public static BNO055IMU imu;
    //    public static IMU imu;
    public static BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    Gamepad driver;

    double maxSpeed = 1.6;

    PIDController controller = new PIDController(.025,.0025,0);

    public Drivetrain(LinearOpMode opMode){
        this.opMode = opMode;

        this.driver = opMode.gamepad1;

        drive = new MecanumDrive(this.opMode.hardwareMap, new Pose2d(0,0,0));

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void run(){
        ChassisSpeeds speeds;

        double thetaPower;

        if(driver.a) {
            thetaPower = controller.calculate(imu.getAngularOrientation().firstAngle, -45 + 180);
        } else if(driver.x){
            thetaPower = controller.calculate(imu.getAngularOrientation().firstAngle, 90);
        } else if(driver.b){
            thetaPower = controller.calculate(imu.getAngularOrientation().firstAngle, -90);
        } else {
            thetaPower = -driver.right_stick_x * Math.PI;
        }

        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                driver.left_stick_y * maxSpeed,
                driver.left_stick_x * maxSpeed,
                thetaPower,
                Rotation2d.fromDegrees(imu.getAngularOrientation().firstAngle + 180)
        );

        MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

        wheelSpeeds.normalize(maxSpeed);

        drive.setWheelPowers(new double[]{
                wheelSpeeds.frontLeftMetersPerSecond,
                wheelSpeeds.rearLeftMetersPerSecond,
                wheelSpeeds.rearRightMetersPerSecond,
                wheelSpeeds.frontRightMetersPerSecond
        });

        if(driver.back){
            drive.setPose(new Pose2d(0,0,0));
            imu.initialize(parameters);
        }
    }
}
