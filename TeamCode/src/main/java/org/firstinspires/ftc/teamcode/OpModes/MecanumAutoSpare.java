package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NewMecanumDriveSpare;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;

@Autonomous
public class MecanumAutoSpare extends LinearOpMode {
    NewMecanumDriveSpare drive;
    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2dWrapper startPose = new Pose2dWrapper(39, -56, Math.toRadians(90));
        Pose2dWrapper movedPose = new Pose2dWrapper(39, -36, Math.toRadians(90));
        drive = new NewMecanumDriveSpare(hardwareMap, startPose.toPose2d());
        imu = hardwareMap.get(IMU.class, "imu");


        telemetry.addData("Yaw: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Pitch: ", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));
        telemetry.addData("Roll: ", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS));
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(startPose.toPose2d())
                        .strafeTo(movedPose.toPose2d().position)
                        .build()
        );
        telemetry.addData("Yaw: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Pitch: ", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));
        telemetry.addData("Roll: ", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS));
        telemetry.update();
       // Actions.runBlocking(drive.actionBuilder());

    }

}
