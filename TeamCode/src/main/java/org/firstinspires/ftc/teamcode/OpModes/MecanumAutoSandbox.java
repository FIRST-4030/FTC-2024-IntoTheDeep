package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.LogFile;
import org.firstinspires.ftc.teamcode.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;

import java.util.Arrays;
import java.util.Locale;

@Config
@Autonomous
public class MecanumAutoSandbox extends LinearOpMode {

    enum Options { TBD, OneRotation, To_Stabilizer, To_Wall }

    Action thisAction;
    boolean inputComplete = false;
    LogFile detailsLog;
    NewMecanumDrive drive;
    Options option = Options.TBD;
    IMU imu;

    public static boolean logDetails = true;

    public static double largeLeft = -40;
    public static double largeForward = 30;
    public static double smallForward = 15;
    public static double shoveLeft = -10;

    public static double midEndX = 60;
    public static double midEndY = -30;
    public static double endX = 60;
    public static double endY = -37;

    public static double startVel = 40.0;
    public static double accelMin = -20.0;
    public static double accelMax = 50.0;
    public static double moveIncrement = 0.0001;
    public static double startX = 7;
    public static double startY = 7;
    public static double startHead = 90;
    public static double step1Head = 90;
    public static double step2Head = 90;

    Double largeMoveLeftX;
    Double largeMoveLeftY;
    Double smallMoveForwardX;
    Double smallMoveForwardY;
    Double shoveLeftX;

    @Override
    public void runOpMode() throws InterruptedException {

        if (logDetails) { detailsLog = new LogFile(LogFile.FileType.Details,"details", "csv"); }

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(startVel),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(accelMin, accelMax);

        TurnConstraints turnConstraints = new TurnConstraints(Math.toRadians(90),Math.toRadians(90),Math.toRadians(90));

        Pose2dWrapper startPose = new Pose2dWrapper( startX, startY, Math.toRadians(startHead));

        imu = hardwareMap.get(IMU.class, "imu");

        drive = new NewMecanumDrive(hardwareMap, startPose.toPose2d(), detailsLog, logDetails);
        if (!drive.controlHub.isMacAddressValid()) {
            drive.controlHub.reportBadMacAddress(telemetry,hardwareMap);
        }

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
                    option = Options.OneRotation;
                    inputTimer.reset();
                }

                if (inputHandler.up("D1:DPAD_UP")) {
                    option = Options.To_Stabilizer;
                    inputTimer.reset();
                }

                if (inputHandler.up("D1:DPAD_DOWN")) {
                    option = Options.To_Wall;
                    inputTimer.reset();
                }
            }

            telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
            telemetry.addLine();
            telemetry.addLine("Options");
            telemetry.addData("   D1:A:","OneRotation");
            telemetry.addData("   D1:DPAD_UP:","To Stabilizer");
            telemetry.addData("   D1:DPAD_DOWN:","To Wall");
            telemetry.addData("Option: ", option);
            telemetry.addData("Press X to finalize values", inputComplete);
            telemetry.update();
        }

        thisAction = drive.actionBuilder(startPose.toPose2d())
                .turn(Math.toRadians(120))
                .build();
        Pose2dWrapper noMovePose = new Pose2dWrapper(startX+moveIncrement, startY, Math.toRadians(step1Head));

        waitForStart();
        if (isStopRequested()) return;

        largeMoveLeftX = startX + largeLeft;
        largeMoveLeftY = startY + largeForward;
        smallMoveForwardX = largeMoveLeftX;
        smallMoveForwardY = largeMoveLeftY + smallForward;
        shoveLeftX = smallMoveForwardX + shoveLeft;

        Pose2dWrapper leftPose = new Pose2dWrapper(largeMoveLeftX, largeMoveLeftY, Math.toRadians(step1Head));
        Pose2dWrapper forwardPose = new Pose2dWrapper(smallMoveForwardX, smallMoveForwardY, Math.toRadians(step2Head));
        Pose2dWrapper shovePose = new Pose2dWrapper(shoveLeftX, smallMoveForwardY, Math.toRadians(step2Head));

        Pose2dWrapper newStartPose = new Pose2dWrapper(0, 0, Math.toRadians(step2Head));
        Pose2dWrapper midEndPose = new Pose2dWrapper(midEndX, midEndY, Math.toRadians(step2Head));
        Pose2dWrapper endPose = new Pose2dWrapper(endX, endY, Math.toRadians(step2Head));

        switch (option) {
            case OneRotation:
                String parY          = String.format(Locale.US, "%.2f",TwoDeadWheelLocalizer.PARAMS.parallelDistance);
                String perpX         = String.format(Locale.US, "%.2f",TwoDeadWheelLocalizer.PARAMS.perpDistance);
                String parallel      = "" + TwoDeadWheelLocalizer.PARAMS.parYTicks;
                String perpendicular = "" + TwoDeadWheelLocalizer.PARAMS.perpXTicks;

                detailsLog.log("Parallel: "+parY+" ("+parallel+"), Perpendicular: "+perpX+" ("+perpendicular+")");

                Action strafeAction = drive.actionBuilder(startPose.toPose2d())
                        .strafeToLinearHeading(noMovePose.toPose2d().position,noMovePose.toPose2d().heading.toDouble())
                        .build();
                Actions.runBlocking(strafeAction);

                detailsLog.logDelta(noMovePose.toPose2d(),drive.pose);
                break;
            case To_Stabilizer:
                detailsLog.log("To Stabilizer");
                detailsLog.log("Start X:"+startPose.toPose2d().position.x);
                detailsLog.log("Start Y:"+startPose.toPose2d().position.y);
                thisAction = drive.actionBuilder(startPose.toPose2d())
                        .strafeToConstantHeading(leftPose.toPose2d().position)
                        .strafeToConstantHeading(forwardPose.toPose2d().position)
                        .strafeToConstantHeading(shovePose.toPose2d().position)
                        .build();
                Actions.runBlocking(thisAction);
                detailsLog.log("Calculated End X:"+shovePose.toPose2d().position.x);
                detailsLog.log("Calculated End Y:"+shovePose.toPose2d().position.y);
                detailsLog.log("End X:"+drive.pose.position.x);
                detailsLog.log("End Y:"+drive.pose.position.y);
                detailsLog.log("End Heading:"+Math.toDegrees(drive.pose.heading.real));
                break;
            case To_Wall:
                imu.resetYaw();
                thisAction = drive.actionBuilder(newStartPose.toPose2d())
                        .strafeToConstantHeading(midEndPose.toPose2d().position)
                        .strafeToConstantHeading(endPose.toPose2d().position)
                        .build();
                Actions.runBlocking(thisAction);
                detailsLog.log("To Wall");
                detailsLog.log("Calculated End X:"+endPose.toPose2d().position.x);
                detailsLog.log("Calculated End Y:"+endPose.toPose2d().position.y);
                detailsLog.log("End X:"+drive.pose.position.x);
                detailsLog.log("End Y:"+drive.pose.position.y);
                detailsLog.log("End Heading:"+Math.toDegrees(drive.pose.heading.real));
                break;
        }
    }
}