package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.LogFile;
import org.firstinspires.ftc.teamcode.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;
import org.firstinspires.ftc.teamcode.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;

import java.util.Arrays;

@Config
@Autonomous
public class MecanumAutoDetails extends LinearOpMode {

    enum Options { TBD, OneStrafeWithConstant, StrafeTurn, Turn }

    LogFile detailsLog;
    NewMecanumDrive drive;
    boolean inputComplete = false;
    Options option = Options.TBD;
    int startX = 0;
    int increment = 0;
    Action thisAction;

    public static boolean logDetails = true;
    public static double baseVel = 40.0;
    public static double accelMin = -20.0;
    public static double accelMax = 50.0;

    @Override
    public void runOpMode() throws InterruptedException {

        if (logDetails) {
            detailsLog = new LogFile("details", "csv");
            detailsLog.logDetailsTitles();
        }

        Pose2dWrapper startPose = new Pose2dWrapper(startX, -55, Math.toRadians(0));

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(baseVel),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(accelMin, accelMax);

        TurnConstraints turnConstraints = new TurnConstraints(Math.toRadians(90),Math.toRadians(90),Math.toRadians(90));

        drive = new NewMecanumDrive(hardwareMap, startPose.toPose2d(), detailsLog, logDetails);

        InputHandler inputHandler;
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime inputTimer = new ElapsedTime();

        runtime.reset();

        inputHandler = InputAutoMapper.normal.autoMap(this);

        while (!inputComplete) {
            inputHandler.loop();
            if (inputTimer.milliseconds() > 250) {

                if (inputHandler.up("D1:X")) {
                    inputComplete = true;
                    inputTimer.reset();
                }

                if (inputHandler.up("D1:A")) {
                    option = Options.OneStrafeWithConstant;
                    inputTimer.reset();
                }

                if (inputHandler.up("D1:B")) {
                    option = Options.StrafeTurn;
                    inputTimer.reset();
                }

                if (inputHandler.up("D1:Y")) {
                    option = Options.Turn;
                    inputTimer.reset();
                }

                if (inputHandler.up("D1:LT")) {
                    increment++;
                    inputTimer.reset();
                }

                if (inputHandler.up("D1:RT")) {
                    increment--;
                    inputTimer.reset();
                }
            }

            telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
            telemetry.addLine();
            telemetry.addData("Option:   ", option);
            telemetry.addData("Increment: ", increment);
            telemetry.addData("Press X to finalize values", inputComplete);
            telemetry.update();
        }

        Pose2dWrapper step1Pose = new Pose2dWrapper(startX + increment, -56, Math.toRadians(120));
        Pose2d step2Pose = new Pose2d(startX + increment, -56,Math.toRadians(120));

        thisAction = drive.actionBuilder(startPose.toPose2d())
                .turn(Math.toRadians(120))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        switch (option) {
             case OneStrafeWithConstant:
                thisAction = drive.actionBuilder(startPose.toPose2d())
                        .strafeToConstantHeading(step1Pose.toPose2d().position)
                        .build();
                detailsLog.log("1,One Strafe");
                Actions.runBlocking(thisAction);
                break;
            case StrafeTurn:
                Action strafeAction = drive.actionBuilder(startPose.toPose2d())
//                        .strafeToConstantHeading(step1Pose.toPose2d().position)
                        .strafeToLinearHeading(step1Pose.toPose2d().position,step1Pose.heading)
                        .build();
//                Pose2d thisPose = new Pose2d (step1Pose.toPose2d().position.x,step1Pose.toPose2d().position.y,Math.toRadians(120));
//                Action strafeTurnAction = drive.actionBuilder(step1Pose.toPose2d())
//                        .turn(thisPose.heading.toDouble())
//                        .build();
//                Action strafeTurnAction = drive.actionBuilder(step1Pose.toPose2d())
//                        .turn(Math.toRadians(60))
//                        .build();
                detailsLog.log("1,Strafe");
                Actions.runBlocking(strafeAction);
                detailsLog.log("1,Turn");
//                Actions.runBlocking(strafeTurnAction);
                break;
            case Turn:
                Action splineAction = drive.actionBuilder(startPose.toPose2d())
//                        .splineTo(step1Pose.toPose2d().position,step1Pose.heading,baseVelConstraint,baseAccelConstraint)
                        .build();
                Action splineTurnAction = drive.actionBuilder(startPose.toPose2d())
                        .turn(Math.toRadians(60))
                        .build();
                detailsLog.log("1,Spline");
                Actions.runBlocking(splineAction);
                detailsLog.log("1,Turn");
                Actions.runBlocking(splineTurnAction);
                break;
        }
        detailsLog.log("1,Done");
    }
}