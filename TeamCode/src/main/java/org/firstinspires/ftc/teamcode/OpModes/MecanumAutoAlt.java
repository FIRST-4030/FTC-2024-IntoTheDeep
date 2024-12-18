package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.canvas.Spline;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.LogFile;
import org.firstinspires.ftc.teamcode.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;

import java.util.Timer;

@Autonomous
public class MecanumAutoAlt extends LinearOpMode {
    public static boolean logDetails = false;

    LogFile detailsLog;
    NewMecanumDrive drive;
    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2dWrapper startPose = new Pose2dWrapper(26, -56, Math.toRadians(90));
        Pose2dWrapper ExperimentalStartPose = new Pose2dWrapper(8, -56, Math.toRadians(90));
        Pose2dWrapper SpecimenCollectionPose = new Pose2dWrapper(8, -34, Math.toRadians(90));
        Pose2dWrapper GoToCollection = new Pose2dWrapper(39, -6, Math.toRadians(90));
        Pose2dWrapper LineUpPushPoseY = new Pose2dWrapper(39, -6, Math.toRadians(90));
        Pose2dWrapper LineUpPushPoseX = new Pose2dWrapper(48, -6, Math.toRadians(90));
        Pose2dWrapper PushPose = new Pose2dWrapper(46, -52, Math.toRadians(90));
        Pose2dWrapper LineUpPushPoseX2 = new Pose2dWrapper(58 , -6, Math.toRadians(90));
        Pose2dWrapper PushPose2 = new Pose2dWrapper(57, -52, Math.toRadians(90));
        Pose2dWrapper LineUpPushPoseX3 = new Pose2dWrapper(63 , -6, Math.toRadians(90));
        Pose2dWrapper PushPose3 = new Pose2dWrapper(61, -52, Math.toRadians(90));
        drive = new NewMecanumDrive(hardwareMap, startPose.toPose2d(), detailsLog, logDetails);
        imu = hardwareMap.get(IMU.class, "imu");


        Telemetry();

        telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
        telemetry.update();

        waitForStart();
        ElapsedTime autoTimer = new ElapsedTime();
        Actions.runBlocking(
                drive.actionBuilder(ExperimentalStartPose.toPose2d())
                        //.strafeTo(ExperimentalStartPose.toPose2d().position)
                        .strafeTo(SpecimenCollectionPose.toPose2d().position)
                        .build()
        );

       /* Actions.runBlocking(
                drive.actionBuilder(SpecimenCollectionPose.toPose2d())
                        .splineToConstantHeading(GoToCollection.toPose2d().position,1)
                        .build()
        );*/

//        Actions.runBlocking(
//                drive.actionBuilder(startPose.toPose2d())
//                        .strafeTo(LineUpPushPoseY.toPose2d().position)
//                        .build()
//        );
//        Telemetry();
//        Actions.runBlocking(
//                drive.actionBuilder(LineUpPushPoseY.toPose2d())
//                    .strafeTo(LineUpPushPoseX.toPose2d().position)
//                    .build()
//        );
//        Telemetry();
//        Actions.runBlocking(
//                drive.actionBuilder(LineUpPushPoseX.toPose2d())
//                        .strafeTo(PushPose.toPose2d().position)
//                        .build()
//        );
//        Telemetry();
//        Actions.runBlocking(
//                drive.actionBuilder(PushPose.toPose2d())
//                    .strafeTo(LineUpPushPoseX.toPose2d().position)
//                    .build()
//        );
//        Telemetry();
//        Actions.runBlocking(
//                drive.actionBuilder(LineUpPushPoseX.toPose2d())
//                        .strafeTo(LineUpPushPoseX2.toPose2d().position)
//                        .build()
//        );
//        Telemetry();
//        Actions.runBlocking(
//                drive.actionBuilder(LineUpPushPoseX2.toPose2d())
//                        .strafeTo(PushPose2.toPose2d().position)
//                        .build()
//        );
//        Telemetry();
//        Actions.runBlocking(
//                drive.actionBuilder(PushPose2.toPose2d())
//                        .strafeTo(LineUpPushPoseX2.toPose2d().position)
//                        .build()
//        );
//        Telemetry();
//        Actions.runBlocking(
//                drive.actionBuilder(LineUpPushPoseX2.toPose2d())
//                        .strafeTo(LineUpPushPoseX3.toPose2d().position)
//                        .build()
//        );
//        Telemetry();
//        Actions.runBlocking(
//                drive.actionBuilder(LineUpPushPoseX3.toPose2d())
//                        .strafeTo(PushPose3.toPose2d().position)
//                        .build()
//        );
//        Telemetry();
//        telemetry.addData("Time: ",autoTimer.seconds());
//        telemetry.update();
//        sleep(2000);
    }

    public void Telemetry(){
        telemetry.addData("Yaw: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Pitch: ", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));
        telemetry.addData("Roll: ", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS));
        telemetry.addData("Vel: ",imu.getRobotAngularVelocity(AngleUnit.RADIANS));
        telemetry.addData("Yaw: ", drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Pitch: ", drive.lazyImu.get().getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));
        telemetry.addData("Roll: ", drive.lazyImu.get().getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS));
        telemetry.addData("Vel: ",drive.lazyImu.get().getRobotAngularVelocity(AngleUnit.RADIANS));
        telemetry.update();
    }
}