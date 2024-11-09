package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;

@Autonomous
public class MecanumAutoSpare extends LinearOpMode {
    NewMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new NewMecanumDrive(hardwareMap, new Pose2d(32,-58,Math.toRadians(90)));

        Pose2dWrapper startPose = new Pose2dWrapper(32, -58, Math.toRadians(90));
        Pose2dWrapper movedPose = new Pose2dWrapper(32, -38, Math.toRadians(90));

        Actions.runBlocking(
                drive.actionBuilder(startPose.toPose2d())
                        .strafeTo(movedPose.toPose2d().position)
                        .build()
        );

    }
}
