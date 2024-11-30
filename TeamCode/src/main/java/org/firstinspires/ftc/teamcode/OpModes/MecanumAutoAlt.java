package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;

@Autonomous
public class MecanumAutoAlt extends LinearOpMode {
    NewMecanumDrive drive;
    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2dWrapper startPose = new Pose2dWrapper(39, -56, Math.toRadians(90));
        Pose2dWrapper LineUpPushPoseY = new Pose2dWrapper(39, -6, Math.toRadians(90));
        Pose2dWrapper LineUpPushPoseX = new Pose2dWrapper(49, -6, Math.toRadians(90));
        Pose2dWrapper PushPose = new Pose2dWrapper(49, -56, Math.toRadians(90));
        Pose2dWrapper LineUpPushPoseX2 = new Pose2dWrapper(59 , -6, Math.toRadians(90));
        Pose2dWrapper PushPose2 = new Pose2dWrapper(59, -6, Math.toRadians(90));
        drive = new NewMecanumDrive(hardwareMap, startPose.toPose2d());
        imu = hardwareMap.get(IMU.class, "imu");


        Telemetry();

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(startPose.toPose2d())
                        .strafeTo(LineUpPushPoseY.toPose2d().position)
                        .build()
        );
        Telemetry();
        Actions.runBlocking(
                drive.actionBuilder(LineUpPushPoseY.toPose2d())
                    .strafeTo(LineUpPushPoseX.toPose2d().position)
                    .build()
        );
        Telemetry();
        sleep(500);
        Actions.runBlocking(
                drive.actionBuilder(LineUpPushPoseX.toPose2d())
                        .strafeTo(PushPose.toPose2d().position)
                        .build()
        );
        Telemetry();
        Actions.runBlocking(
                drive.actionBuilder(PushPose.toPose2d())
                    .strafeTo(LineUpPushPoseX.toPose2d().position)
                    .build()
        );
        Telemetry();
        Actions.runBlocking(
                drive.actionBuilder(LineUpPushPoseX.toPose2d())
                        .strafeTo(LineUpPushPoseX2.toPose2d().position)
                        .build()
        );
        Telemetry();
        Actions.runBlocking(
                drive.actionBuilder(LineUpPushPoseX2.toPose2d())
                        .strafeTo(PushPose2.toPose2d().position)
                        .build()
        );
        Telemetry();
    }

    public void Telemetry(){
        telemetry.addData("Yaw: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Pitch: ", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));
        telemetry.addData("Roll: ", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS));
        telemetry.update();
    }
}
