package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.LinearMotorController;
import org.firstinspires.ftc.teamcode.LogFile;
import org.firstinspires.ftc.teamcode.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;

@Config
@Autonomous(name = "MecanumAuto")
public class MecanumAuto extends LinearOpMode {

    InputHandler inputHandler;
    LinearMotorController liftRotation;
    LinearMotorController liftExtension;
    Servo clawServo, wrist, wristRotation;
    ElapsedTime runtime = new ElapsedTime();
    boolean inputComplete = false;
    boolean side = true;
    boolean experimental = false;
    public Pose2dWrapper startPose;
    public Pose2dWrapper pushPrep1;
    public Pose2dWrapper pushPrep2;
    public Pose2dWrapper pushPrep3;
    public Pose2dWrapper brickPush;
    public Pose2dWrapper collectionPose;
    public Pose2dWrapper collectionPose2;
    public Pose2dWrapper collectionPose3;
    public Pose2dWrapper intermediaryPose;
    public Pose2dWrapper depositPose;
    public Pose2dWrapper depositPose2;
    public Pose2dWrapper depositPose3;
    public Pose2dWrapper depositPose4;
    public Pose2dWrapper preParkPose;
    public Pose2dWrapper parkPose;
    public Pose2dWrapper brickPush2;
    public Pose2dWrapper pushPrep4;
    public Pose2dWrapper pushPrep5;
    public Pose2dWrapper fifthSample;
    public static boolean logSampleSide = true;
    public static boolean logSpecimenSide = false;
    public static boolean logDetails = true;

    LogFile sampleSideLog;
    LogFile specimenSideLog;
    LogFile detailsLog;

    /**
     * Runs though linear OpMode once, initializes, waits for user input, and performs a linear sequence
     * based on the initialization inputs. Designed for 2024 Into The Deep.
     *
     * @throws InterruptedException When giving a command to stop in the middle of a while loop, an exception is thrown
     */
    @Override
    public void runOpMode() throws InterruptedException {

        if (logSampleSide) { sampleSideLog = new LogFile("sample", "csv" ); }
        if (logSampleSide) { sampleSideLog.logSampleTitles(); }

        if (logSpecimenSide) { specimenSideLog = new LogFile("specimen", "csv" ); }
        if (logSpecimenSide) { specimenSideLog.logSampleTitles(); }

        if (logDetails) { detailsLog = new LogFile("details", "csv" ); }
        if (logDetails) { detailsLog.logDetailsTitles(); }

        runtime.reset();
        liftExtension = new LinearMotorController(hardwareMap, "slide",
                1390, true, true);
        liftRotation = new LinearMotorController(hardwareMap, "swing",
                3000, false,true);
        wrist = hardwareMap.get(Servo.class, "wrist");
        clawServo = hardwareMap.get(Servo.class, "claw");
        wristRotation = hardwareMap.get(Servo.class, "wristRotation");
        inputHandler = InputAutoMapper.normal.autoMap(this);
        clawServo.setPosition(0.95);
        wristRotation.setPosition(0.5);

        while (!inputComplete) {
            inputHandler.loop();
            if (inputHandler.up("D1:X")) {
                inputComplete = true;
            }
            if(inputHandler.up("D1:Y")) {
                experimental = !experimental;
            }
            if(inputHandler.up("D1:RT")){
                side = !side;
            }
            telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
            telemetry.addData("-----Initialization-----", "");
            telemetry.addLine();
            telemetry.addData("Auto Mode: ", side ? "Sample Scoring" : "Specimen Scoring");
            telemetry.addData("Experimental Mode? ", experimental ? "Enabled" : "Disabled");
            telemetry.addData("Press X to finalize values", inputComplete);
            telemetry.update();

        }
        if(!side) {
            ///Positions for when scoring on Specimen Side
            startPose = new Pose2dWrapper(15.8, -55.75, 1.5708);
            depositPose = new Pose2dWrapper(15.8, -27.5, 1.5708);
            pushPrep1 = new Pose2dWrapper(40.8, -30.75, 1.5708);
            pushPrep2 = new Pose2dWrapper(39.8, -7.75, 1.5708);
            pushPrep3 = new Pose2dWrapper(52.8, -7.75, 1.5708);
            brickPush = new Pose2dWrapper(51.8, -51.75, 1.5708);
            collectionPose = new Pose2dWrapper(46.5, -66.5, 0);
            intermediaryPose = new Pose2dWrapper(43, -63.5, 1.5708);
            depositPose2 = new Pose2dWrapper(9.8, -27.5, 1.5708);
            depositPose3 = new Pose2dWrapper(4.8, -27.5, 1.5708);
            parkPose = new Pose2dWrapper(51.8, -51.75, 1.5708);
            depositPose4 = new Pose2dWrapper(0, -27.5, 1.5708);
            ///Experimental Auto-Code, optimized 4-Specimen auto pathing
                if(experimental){
                    startPose = new Pose2dWrapper(15.8, -55.75, 4.7123);
                    depositPose = new Pose2dWrapper(1.8, -28.5, 4.7123);
                    pushPrep1 = new Pose2dWrapper(40.8, -26.75, 4.7123);
                    pushPrep2 = new Pose2dWrapper(40.8, -20, 4.7123);
                    pushPrep3 = new Pose2dWrapper(59, -15, 4.7123);
                    brickPush = new Pose2dWrapper(59, -38.75, 4.7123);
                    pushPrep4 = new Pose2dWrapper(54.8, -12.5, 4.7123);
                    pushPrep5 = new Pose2dWrapper(58, -8.5, 4.7123);
                    brickPush2 = new Pose2dWrapper(66.5, -44, 4.7123);
                    collectionPose = new Pose2dWrapper(44.5, -46.5, 4.7123);
                    intermediaryPose = new Pose2dWrapper(15.8, -31.75, 4.7123);
                    depositPose2 = new Pose2dWrapper(5.8, -28.5, 4.7123);
                    depositPose3 = new Pose2dWrapper(9.8, -28.5, 4.7123);
                    depositPose4 = new Pose2dWrapper(10.8, -28.5, 4.7123);
                    parkPose = new Pose2dWrapper(51.8, -51.75, 4.7123);

                }
        } else {
            ///Positions for when scoring on Sample Side
            startPose = new Pose2dWrapper(-32, -56, 1.5708);
            depositPose = new Pose2dWrapper(-61, -55.5, Math.toRadians(60));
            collectionPose = new Pose2dWrapper(-49, -35, 1.5708);
            collectionPose2 = new Pose2dWrapper(-58.5, -35, 1.5708);
            collectionPose3 = new Pose2dWrapper(-62.75, -23, Math.toRadians(125));
            preParkPose = new Pose2dWrapper(-42, -18, 0);
            parkPose = new Pose2dWrapper(-38, -18, 0);
            fifthSample = new Pose2dWrapper(-56, 28, 0);
        }

        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap, startPose.toPose2d(), detailsLog, logDetails);

        ///START AUTO:
        waitForStart();
        if (isStopRequested()) return;

        liftExtension.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRotation.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftExtension.getLiftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRotation.getLiftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /// Auto code for when scoring Specimens
        if(!side) {
            if(experimental){
                ///Cycle 1
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Cycle 1", startPose.toPose2d() ); }
                Actions.runBlocking(experimentalPreScore());
                Actions.runBlocking(
                        drive.actionBuilder(startPose.toPose2d())
                                .strafeTo(depositPose.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", depositPose.toPose2d() ); }
                Actions.runBlocking(extendScore());
                sleep(400);
                Actions.runBlocking(openClaw());
                sleep(250);
                ///Cycle 2
                Actions.runBlocking(experimentalCollectionPrep());
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Cycle 2", depositPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                //.splineToConstantHeading(intermediaryPose.toPose2d().position, intermediaryPose.toPose2d().heading)
                                .setTangent(0)
                                .splineToConstantHeading(pushPrep1.toPose2d().position, 1.5708)
                                .splineToConstantHeading(pushPrep2.toPose2d().position, 1.5708)
                                .splineToConstantHeading(pushPrep3.toPose2d().position, 4.7123)
                                .splineToConstantHeading(brickPush.toPose2d().position, brickPush.toPose2d().heading)
                                .splineToConstantHeading(pushPrep4.toPose2d().position, 1.5708)
                                .splineToConstantHeading(pushPrep5.toPose2d().position, 4.7123)
                                .splineToConstantHeading(brickPush2.toPose2d().position, brickPush2.toPose2d().heading)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", brickPush2.toPose2d() ); }
                /*
                Actions.runBlocking(
                        drive.actionBuilder(brickPush.toPose2d())
                                .setTangent(1.5708)
                                .splineToConstantHeading(pushPrep4.toPose2d().position, 1.5708)
                                .splineToConstantHeading(pushPrep5.toPose2d().position, 3.1415*2)
                                .splineToConstantHeading(brickPush2.toPose2d().position, brickPush2.toPose2d().heading)
                                .build()
                );
                */
                /*Actions.runBlocking(
                        drive.actionBuilder(intermediaryPose.toPose2d())
                                .strafeTo(pushPrep1.toPose2d().position)
                                .build()
                );
                Actions.runBlocking(
                        drive.actionBuilder(pushPrep1.toPose2d())
                                .strafeTo(pushPrep2.toPose2d().position)
                                .build()
                );
                Actions.runBlocking(
                        drive.actionBuilder(pushPrep2.toPose2d())
                                .strafeTo(pushPrep3.toPose2d().position)
                                .build()
                );
                Actions.runBlocking(
                        drive.actionBuilder(pushPrep3.toPose2d())
                                .strafeTo(brickPush.toPose2d().position)
                                .build()
                );
                Actions.runBlocking(
                        drive.actionBuilder(brickPush.toPose2d())
                                .strafeTo(pushPrep4.toPose2d().position)
                                .build()
                );
                Actions.runBlocking(
                        drive.actionBuilder(pushPrep4.toPose2d())
                                .strafeTo(pushPrep5.toPose2d().position)
                                .build()
                );
                Actions.runBlocking(
                        drive.actionBuilder(pushPrep5.toPose2d())
                                .strafeTo(brickPush2.toPose2d().position)
                                .build()
                );*/
                /*Actions.runBlocking(
                        drive.actionBuilder(brickPush2.toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );

                 */
                Actions.runBlocking(experimentalExtend());
                sleep(150);
                Actions.runBlocking(collect());
                sleep(300);
                Actions.runBlocking(experimentalPreScore());
                sleep(100);
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Deposit 2", brickPush2.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(brickPush2.toPose2d())
                                .strafeTo(depositPose2.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", depositPose2.toPose2d() ); }
                Actions.runBlocking(
                        extendScore()
                );
                sleep(400);
                Actions.runBlocking(openClaw());
                sleep(250);
                Actions.runBlocking(experimentalCollectionPrep());
                ///Cycle 3
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Cycle 3", depositPose2.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose2.toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( true, "", collectionPose.toPose2d() ); }
                Actions.runBlocking(experimentalExtend());
                sleep(150);
                Actions.runBlocking(collect());
                sleep(300);
                Actions.runBlocking(experimentalPreScore());
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Deposit 3", collectionPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeTo(depositPose3.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", depositPose3.toPose2d() ); }
                Actions.runBlocking(extendScore());
                sleep(400);
                Actions.runBlocking(openClaw());
                sleep(250);
                ///Cycle 4
                Actions.runBlocking(experimentalCollectionPrep());
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Cycle 4", depositPose3.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose3.toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", collectionPose.toPose2d() ); }
                Actions.runBlocking(experimentalExtend());
                sleep(150);
                Actions.runBlocking(collect());
                sleep(300);
                Actions.runBlocking(experimentalPreScore());
                sleep(100);
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Deposit 4", collectionPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeTo(depositPose4.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", depositPose4.toPose2d() ); }
                Actions.runBlocking(extendScore());
                sleep(400);
                Actions.runBlocking(openClaw());
                sleep(300);
                Actions.runBlocking(stow());
                if (logSampleSide) { sampleSideLog.logSample( true, "Park", depositPose4.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose4.toPose2d())
                                .strafeTo(parkPose.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", parkPose.toPose2d() ); }
            } else {
                ///Cycle 1
                Actions.runBlocking(preScore());

                if (logSpecimenSide) { specimenSideLog.logSample( true, "Cycle 1", startPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(startPose.toPose2d())
                                .strafeTo(depositPose.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", depositPose.toPose2d() ); }
                Actions.runBlocking(armDown());
                sleep(500);
                Actions.runBlocking(openClaw());
                sleep(500);
                ///Cycle 2
                Actions.runBlocking(stow());
                sleep(500);
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Push Prep 1", depositPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeTo(pushPrep1.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", pushPrep1.toPose2d() ); }
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Push Prep 2", pushPrep1.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(pushPrep1.toPose2d())
                                .strafeTo(pushPrep2.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", pushPrep2.toPose2d() ); }
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Push Prep 3", pushPrep2.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(pushPrep2.toPose2d())
                                .strafeTo(pushPrep3.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", pushPrep3.toPose2d() ); }
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Brick Push", pushPrep3.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(pushPrep3.toPose2d())
                                .strafeTo(brickPush.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", brickPush.toPose2d() ); }
                Actions.runBlocking(collectionPrep());
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Collection", brickPush.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(brickPush.toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", collectionPose.toPose2d() ); }
                sleep(400);
                Actions.runBlocking(extend());
                sleep(300);
                Actions.runBlocking(collect());
                sleep(300);
                Actions.runBlocking(preScore());
                sleep(500);
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Intermediary", collectionPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeToLinearHeading(intermediaryPose.toPose2d().position, intermediaryPose.heading)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", intermediaryPose.toPose2d() ); }
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Deposit 2", intermediaryPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(intermediaryPose.toPose2d())
                                .strafeTo(depositPose2.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", depositPose2.toPose2d() ); }
                sleep(500);
                Actions.runBlocking(
                        armDown()
                );
                sleep(500);
                Actions.runBlocking(openClaw());
                sleep(500);
                ///Cycle 3
                Actions.runBlocking(stow());
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Collection", depositPose2.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose2.toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", collectionPose.toPose2d() ); }
                sleep(500);
                Actions.runBlocking(collectionPrep());
                sleep(500);
                Actions.runBlocking(extend());
                sleep(500);
                Actions.runBlocking(collect());
                sleep(500);
                Actions.runBlocking(preScore());
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Intermediary", collectionPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeToLinearHeading(intermediaryPose.toPose2d().position, intermediaryPose.heading)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", intermediaryPose.toPose2d() ); }
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Deposit 3", intermediaryPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(intermediaryPose.toPose2d())
                                .strafeTo(depositPose3.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", depositPose3.toPose2d() ); }
                sleep(300);
                Actions.runBlocking(armDown());
                sleep(500);
                Actions.runBlocking(openClaw());
                ///Cycle 4
                sleep(250);
                Actions.runBlocking(stow());
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Collection", depositPose3.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose3.toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", collectionPose.toPose2d() ); }
                sleep(500);
                Actions.runBlocking(collectionPrep());
                Actions.runBlocking(extend());
                sleep(500);
                Actions.runBlocking(collect());
                sleep(500);
                Actions.runBlocking(preScore());
                sleep(500);
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Intermediary", collectionPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeToLinearHeading(intermediaryPose.toPose2d().position, intermediaryPose.heading)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", intermediaryPose.toPose2d() ); }
                sleep(500);
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Deposit 4", intermediaryPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(intermediaryPose.toPose2d())
                                .strafeTo(depositPose4.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", depositPose4.toPose2d() ); }
                Actions.runBlocking(armDown());
                sleep(500);
                Actions.runBlocking(openClaw());
                sleep(500);
                Actions.runBlocking(stow());
                if (logSpecimenSide) { specimenSideLog.logSample( true, "Park", depositPose4.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose4.toPose2d())
                                .strafeTo(parkPose.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logSample( false, "", parkPose.toPose2d() ); }
            }
        } else {
            if (!experimental) {
                /// Auto code for when scoring Samples

                ///Cycle 1
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logSample( true, "Cycle 1", startPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(startPose.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", depositPose.toPose2d() ); }
                sleep(500);
                Actions.runBlocking(openClaw());
                ///Cycle 2
                if (logSampleSide) { sampleSideLog.logSample( true, "Cycle 2", depositPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", collectionPose.toPose2d() ); }
                Actions.runBlocking(floorPickUpPrep());
                sleep(1500);
                Actions.runBlocking(pickUp());
                sleep(300);
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logSample( true, "Deposit", collectionPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", depositPose.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(openClaw());
                sleep(500);
                if (logSampleSide) { sampleSideLog.logSample( true, "Collection 2", depositPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(collectionPose2.toPose2d().position, collectionPose2.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", collectionPose2.toPose2d() ); }
                Actions.runBlocking(floorPickUpPrep());
                sleep(1500);
                Actions.runBlocking(pickUp());
                sleep(500);
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logSample( true, "Deposit", collectionPose2.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose2.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", depositPose.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(openClaw());
                sleep(300);
                Actions.runBlocking(lastFloorPickUpPrep());
                if (logSampleSide) { sampleSideLog.logSample( true, "Collection 3", depositPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(collectionPose3.toPose2d().position, collectionPose3.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", collectionPose3.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(lastFloorPickUp1());
                sleep(300);
                Actions.runBlocking(lastFloorPickUp2());
                sleep(500);
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logSample( true, "Deposit", collectionPose3.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose3.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", depositPose.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(openClaw());
                sleep(500);
                Actions.runBlocking(armParkPose());
                if (logSampleSide) { sampleSideLog.logSample( true, "Pre-Park", depositPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(preParkPose.toPose2d().position, preParkPose.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", preParkPose.toPose2d() ); }
                if (logSampleSide) { sampleSideLog.logSample( true, "Park", preParkPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(preParkPose.toPose2d())
                                .strafeToLinearHeading(parkPose.toPose2d().position, preParkPose.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", parkPose.toPose2d() ); }
                sleep(1500);
            } else {
                /// Auto code for when scoring Samples

                ///Cycle 1
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logSample( true, "Cycle 1", startPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(startPose.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", depositPose.toPose2d() ); }
                sleep(500);
                Actions.runBlocking(openClaw());
                ///Cycle 2
                Actions.runBlocking(floorPickUpPrep());
                if (logSampleSide) { sampleSideLog.logSample( true, "Sample 5", depositPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(fifthSample.toPose2d().position, fifthSample.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", fifthSample.toPose2d() ); }
                sleep(500);
                Actions.runBlocking(pickUp());
                sleep(300);
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logSample( true, "Deposit", fifthSample.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(fifthSample.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", depositPose.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(openClaw());
                sleep(500);
                if (logSampleSide) { sampleSideLog.logSample( true, "Collection", depositPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", collectionPose.toPose2d() ); }
                Actions.runBlocking(floorPickUpPrep());
                sleep(700);
                Actions.runBlocking(pickUp());
                sleep(500);
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logSample( true, "Deposit", collectionPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", depositPose.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(openClaw());
                sleep(300);
                Actions.runBlocking(floorPickUpPrep());
                if (logSampleSide) { sampleSideLog.logSample( true, "Collection 2", depositPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(collectionPose2.toPose2d().position, collectionPose2.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", collectionPose2.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(pickUp());
                sleep(500);
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logSample( true, "Deposit", collectionPose2.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose2.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", depositPose.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(openClaw());
                sleep(300);
                Actions.runBlocking(lastFloorPickUpPrep());
                if (logSampleSide) { sampleSideLog.logSample( true, "Collection 3", depositPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(collectionPose3.toPose2d().position, collectionPose3.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", collectionPose3.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(lastFloorPickUp1());
                sleep(300);
                Actions.runBlocking(lastFloorPickUp2());
                sleep(500);
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logSample( true, "Deposit", collectionPose2.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose2.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", depositPose.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(openClaw());
                sleep(500);
                Actions.runBlocking(armParkPose());
                if (logSampleSide) { sampleSideLog.logSample( true, "Pre-Park", depositPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(preParkPose.toPose2d().position, preParkPose.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", preParkPose.toPose2d() ); }
                if (logSampleSide) { sampleSideLog.logSample( true, "Park", preParkPose.toPose2d() ); }
                Actions.runBlocking(
                        drive.actionBuilder(preParkPose.toPose2d())
                                .strafeToLinearHeading(parkPose.toPose2d().position, preParkPose.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logSample( false, "", parkPose.toPose2d() ); }
                sleep(1500);
            }
        }
    }

    /**
     * Sets the arm and extension to the required position for scoring a specimen.
     *
     * @return an action to be run, parsable by RoadRunner
     */
    public Action preScore() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftRotation.setTarget(2000);
                liftExtension.setTarget(120);
                wrist.setPosition(0.4);
                clawServo.setPosition(0.95);
                return liftRotation.getLiftMotor().getCurrentPosition() > 1960;
            }
        };
    }


    public Action experimentalPreScore() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                wrist.setPosition(0.91);
                clawServo.setPosition(0.975);
                liftRotation.setTarget(2955);
                liftExtension.setTarget(0);
                return liftRotation.getLiftMotor().getCurrentPosition() > 2960;
            }
        };
    }

    public Action scoreSpecimen() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftExtension.setTarget(200);
                return timer.milliseconds() > 300;
            }
        };
    }

    /**
     * Moves the arm down to clip the specimen on the bar
     * pre: arm is in preScore position.
     *
     * @return an action to be run, parsable by RoadRunner
     */
    public Action armDown() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                    liftRotation.setTarget(1332);
                    return liftRotation.getLiftMotor().getCurrentPosition() <= 1350;
            }
        };
    }

    /**
     * Opens the claw, general use but mainly used after armDown to release the specimen and score.
     *
     * @return an action to be run, parsable by RoadRunner
     */
    public Action openClaw() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                    ElapsedTime timer = new ElapsedTime();
                    clawServo.setPosition(0.3);
                    return timer.milliseconds() > 300;
            }
        };
    }


    /**
     * General use, resets the arm position to be out of the way, and safe to transition into TeleOp
     *
     * @return an action to be run, parsable by RoadRunner
     */
    public Action stow() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftExtension.setTarget(0);
                liftRotation.setTarget(0);
                wrist.setPosition(1);
                clawServo.setPosition(0.95);
                return liftRotation.getLiftMotor().getCurrentPosition() <= 300;
            }
        };
    }

    /**
     * Moves are to be parallel to the ground, ready to collect a specimen from the field wall.
     *
     * @return an action to be run, parsable by RoadRunner
     */
    public Action collectionPrep() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftRotation.setTarget(850);
                clawServo.setPosition(0.3);
                wrist.setPosition(0.5);
                return timer.milliseconds() > 1100;
            }
        };
    }

    public Action experimentalCollectionPrep() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftRotation.setTarget(815);
                liftExtension.setTarget(0);
                clawServo.setPosition(0.3);
                wrist.setPosition(0.5);
                return timer.milliseconds() > 1100;
            }
        };
    }

    /**
     * Extends the arm into the wall to ensure the claw is around a specimen when collection from the wall
     * pre: arm is in collectionPrep pose
     *
     * @return an action to be run, parsable by RoadRunner
     */
    public Action extend() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftExtension.setTarget(460);
                return timer.milliseconds() >= 200;
            }
        };
    }

    public Action experimentalExtend() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftExtension.setTarget(405);
                return timer.milliseconds() >= 200;
            }
        };
    }

    public Action extendScore() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftExtension.setTarget(450);
                return timer.milliseconds() >= 300;
            }
        };
    }

    /**
     * General use, closes the claw no matter the position, generally used for collecting a specimen
     *
     * @return an action to be run, parsable by RoadRunner
     */
    public Action collect() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                clawServo.setPosition(1);
                return timer.milliseconds() > 300;
            }
        };
    }

    /**
     * Moves the arm to the vertical position and extends the arm to maximum extension, used for scoring samples
     *
     * @return an action to be run, parsable by RoadRunner
     */
    public Action highBucketPrep() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                wrist.setPosition(0.775);
                clawServo.setPosition(0.95);
                liftRotation.setTarget(2970);
                liftExtension.setTarget(1390);
                return timer.milliseconds() > 1200;
            }
        };
    }

    /**
     * Moves the arm to just above the floor, wrist down, claw open, and slightly extended.
     * Used to collect samples off of the group.
     *
     * @return an action to be run, parsable by RoadRunner
     */
    public Action floorPickUpPrep() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftRotation.setTarget(470);
                liftExtension.setTarget(530);
                wrist.setPosition(0.15);
                return timer.milliseconds() > 500;
            }

        };
    }

    public Action lastFloorPickUpPrep() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftRotation.setTarget(820);
                liftExtension.setTarget(450);
                wrist.setPosition(0.15);
                return timer.milliseconds() > 500;
            }

        };
    }

    public Action lastFloorPickUp1() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftRotation.setTarget(470);
                return timer.milliseconds() > 500;
            }

        };
    }

    public Action lastFloorPickUp2() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                clawServo.setPosition(0.95);
                return timer.milliseconds() > 500;
            }

        };
    }

    /**
     * Moves arm down and closes claw to pick up a Sample off the ground
     * pre: arm is in floorPickUpPrep position.
     *
     * @return an action to be run, parsable by RoadRunner
     */
    public Action pickUp() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                clawServo.setPosition(0.95);
                liftRotation.setTarget(470);
                return timer.milliseconds() > 500;
            }
        };
    }

    /**
     * Moves the arm to a position where it will touch the low bar when parking, securing parking points in auto
     *
     * @return an action to be run, parsable by RoadRunner
     */
    public Action armParkPose() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftRotation.setTarget(1595);
                liftExtension.setTarget(0);
                wrist.setPosition(0.45);
                return timer.milliseconds() >= 700;
            }
        };
    }


}
