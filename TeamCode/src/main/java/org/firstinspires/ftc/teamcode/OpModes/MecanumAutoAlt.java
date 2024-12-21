package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.canvas.Spline;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.LogFile;
import org.firstinspires.ftc.teamcode.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;

import java.util.Timer;
@Config
@Autonomous
public class MecanumAutoAlt extends LinearOpMode {
    public static boolean logDetails = false;
    public static double experimentalStartX = 8;
    public static double experimentalStartY = -56;
    public static double specimenCollectionX = 8;
    public static double specimenCollectionY = -28;
    public static double splineSetUpX = 32;
    public static double splineSetUpY = -48;
    public static double goToCollectionX = 39;
    public static double goToCollectionY = -6;
    public static double goToCollectionTan = 1;

    LogFile detailsLog;
    NewMecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2dWrapper startPose = new Pose2dWrapper(34, -56, Math.toRadians(90));
        Pose2dWrapper ExperimentalStartPose = new Pose2dWrapper(experimentalStartX, experimentalStartY, Math.toRadians(90));
        Pose2dWrapper SpecimenCollectionPose = new Pose2dWrapper(specimenCollectionX, specimenCollectionY, Math.toRadians(90));
        Pose2dWrapper SplineSetUpPose = new Pose2dWrapper(splineSetUpX, splineSetUpY, Math.toRadians(90));
        Pose2dWrapper GoToCollection = new Pose2dWrapper(goToCollectionX, goToCollectionY, Math.toRadians(90));
        Pose2dWrapper LineUpPushPoseY = new Pose2dWrapper(39, -6, Math.toRadians(90));
        Pose2dWrapper LineUpPushPoseX = new Pose2dWrapper(48, -6, Math.toRadians(90));
        Pose2dWrapper PushPose = new Pose2dWrapper(46, -52, Math.toRadians(90));
        Pose2dWrapper LineUpPushPoseX2 = new Pose2dWrapper(58 , -6, Math.toRadians(90));
        Pose2dWrapper PushPose2 = new Pose2dWrapper(57, -52, Math.toRadians(90));
        Pose2dWrapper LineUpPushPoseX3 = new Pose2dWrapper(63 , -6, Math.toRadians(90));
        Pose2dWrapper PushPose3 = new Pose2dWrapper(61, -52, Math.toRadians(90));
        drive = new NewMecanumDrive(hardwareMap, ExperimentalStartPose.toPose2d(), detailsLog, logDetails);

        Telemetry();

        telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
        telemetry.update();

        waitForStart();
        ElapsedTime autoTimer = new ElapsedTime();
        Actions.runBlocking(
                drive.actionBuilder(ExperimentalStartPose.toPose2d())
                        .strafeTo(SpecimenCollectionPose.toPose2d().position)
                        .build()
        );
        Actions.runBlocking(
                drive.actionBuilder(SpecimenCollectionPose.toPose2d())
                        .strafeTo(SplineSetUpPose.toPose2d().position)
                        .build()
        );
        //Actions.runBlocking(
        //        drive.actionBuilder(SplineSetUpPose.toPose2d())
        //                .splineToConstantHeading(GoToCollection.toPose2d().position,goToCollectionTan)
        //                .build()
        //);

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
        telemetry.addData("Yaw: ", drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Pitch: ", drive.lazyImu.get().getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));
        telemetry.addData("Roll: ", drive.lazyImu.get().getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS));
        telemetry.addData("Vel: ",drive.lazyImu.get().getRobotAngularVelocity(AngleUnit.RADIANS));
        telemetry.update();
    }
}