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
    public Pose2dWrapper stepPose;
    public Pose2dWrapper anchorPose;
    public Pose2dWrapper anchorPose2;
    public Pose2dWrapper anchorPose3;
    public Pose2dWrapper anchorPose4;
    public Pose2dWrapper depositPose5;
    public double increment = 0.00000001;
    public static boolean logSampleSide = false;
    public static boolean logSpecimenSide = false;
    public static boolean logDetails = false;

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

        if (logSampleSide) { sampleSideLog = new LogFile(LogFile.FileType.Action, "sample", "csv" ); }

        if (logSpecimenSide) { specimenSideLog = new LogFile(LogFile.FileType.Action, "specimen", "csv" ); }

        if (logDetails) { detailsLog = new LogFile(LogFile.FileType.Details, "details", "csv" ); }

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
        wristRotation.setPosition(0.39);

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
            startPose = new Pose2dWrapper(15.8, -55.75, 4.7123);
            stepPose = new Pose2dWrapper(15.9, -55.75, 1.5708);
            depositPose = new Pose2dWrapper(1.8, -27.3, 4.7123);
            pushPrep1 = new Pose2dWrapper(40.8, -26.75, 4.7123);
            pushPrep2 = new Pose2dWrapper(40.8, -20, 4.7123);
            pushPrep3 = new Pose2dWrapper(58, -15, 4.7123);
            brickPush = new Pose2dWrapper(58, -40.5, 4.7123);
            pushPrep4 = new Pose2dWrapper(47, -24.5, 4.7123); //FIX
            pushPrep5 = new Pose2dWrapper(69, -24.5, 4.7123);
            brickPush2 = new Pose2dWrapper(65, -44, 4.7123);
            collectionPose = new Pose2dWrapper(44.5, -46.5, 4.7123);
            intermediaryPose = new Pose2dWrapper(15.8, -31.75, 4.7123);
            depositPose2 = new Pose2dWrapper(2.3, -28.5, 4.7123);
            depositPose3 = new Pose2dWrapper(9.8, -28.5, 4.7123);
            depositPose4 = new Pose2dWrapper(10.8, -28.5, 4.7123);
            parkPose = new Pose2dWrapper(51.8, -51.75, 4.7123);
            ///Experimental Auto-Code, optimized 4-Specimen auto pathing
                if(experimental){
                    //TODO: Change to -55.75 if not driving on home field red side
                    startPose = new Pose2dWrapper(15.8, -55.15, 4.7123);
                    anchorPose = new Pose2dWrapper(43, -33, Math.toRadians(45));
                    anchorPose2 = new Pose2dWrapper(50.75, -31.5, Math.toRadians(45));
                    anchorPose3 = new Pose2dWrapper(63, -32.5, Math.toRadians(45));
                    anchorPose4 = new Pose2dWrapper(43, -37.5, Math.toRadians(300));
                    depositPose = new Pose2dWrapper(13, -27, 4.7123);
                    collectionPose = new Pose2dWrapper(44.5, -46.5, 4.7123);
                    intermediaryPose = new Pose2dWrapper(15.8, -31.75, 4.7123);
                    depositPose2 = new Pose2dWrapper(12, -26.8, 4.7123);
                    depositPose3 = new Pose2dWrapper(12, -26.8, 4.7123);
                    depositPose4 = new Pose2dWrapper(12, -26.8, 4.7123);
                    depositPose5 = new Pose2dWrapper(12, -26.8, 4.7123);
                    parkPose = new Pose2dWrapper(51.8, -51.75, 4.7123);
                }
        } else {
            ///Positions for when scoring on Sample Side
            startPose = new Pose2dWrapper(-32, -56, 1.5708);
            stepPose = new Pose2dWrapper(15.9, -55.75, 1.5708);
            depositPose = new Pose2dWrapper(-61.5, -47, Math.toRadians(60));
            collectionPose = new Pose2dWrapper(-49, -35, 1.5708);
            collectionPose2 = new Pose2dWrapper(-58.5, -35, 1.5708);
            collectionPose3 = new Pose2dWrapper(-59.5, -30.25, Math.toRadians(125));
            preParkPose = new Pose2dWrapper(-42, -8.5, 0);
            parkPose = new Pose2dWrapper(-24.5, -8.5, 0);
        }

        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap, startPose.toPose2d(), detailsLog,true);

        ///START AUTO:
        waitForStart();
        if (isStopRequested()) return;

        liftExtension.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRotation.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftExtension.getLiftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRotation.getLiftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /// Auto code for when scoring Specimens
        if(!side) {
            if(!experimental){
                ///Cycle 1
                if (logSpecimenSide) { specimenSideLog.logAction( startPose.toPose2d(), "Cycle 1" ); }
                Actions.runBlocking(experimentalPreScore());
                Actions.runBlocking(
                        drive.actionBuilder(startPose.toPose2d())
                                .strafeTo(depositPose.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose.toPose2d() ); }
                Actions.runBlocking(extendScore());
                sleep(400);
                Actions.runBlocking(openClaw());
                sleep(250);
                ///Cycle 2
                Actions.runBlocking(experimentalCollectionPrep());
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose.toPose2d(), "Cycle 2" ); }
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
                if (logSpecimenSide) { specimenSideLog.logAction( brickPush2.toPose2d() ); }
                sleep(350);
                Actions.runBlocking(experimentalExperimentalExtend());
                sleep(600);
                Actions.runBlocking(collect());
                sleep(350);
                Actions.runBlocking(experimentalPreScore());
                sleep(100);
                if (logSpecimenSide) { specimenSideLog.logAction( brickPush2.toPose2d(), "Deposit 2" ); }
                Actions.runBlocking(
                        drive.actionBuilder(brickPush2.toPose2d())
                                .strafeTo(depositPose2.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose2.toPose2d(), "" ); }
                Actions.runBlocking(
                        extendScore()
                );
                sleep(400);
                Actions.runBlocking(openClaw());
                sleep(250);
                Actions.runBlocking(experimentalCollectionPrep());
                ///Cycle 3
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose2.toPose2d(), "Cycle 3" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose2.toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( collectionPose.toPose2d() ); }
                Actions.runBlocking(experimentalExtend());
                sleep(275);
                Actions.runBlocking(collect());
                sleep(300);
                Actions.runBlocking(experimentalPreScore());
                if (logSpecimenSide) { specimenSideLog.logAction( collectionPose.toPose2d(), "Deposit 3" ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeTo(depositPose3.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose3.toPose2d() ); }
                Actions.runBlocking(extendScore());
                sleep(400);
                Actions.runBlocking(openClaw());
                sleep(250);
                ///Cycle 4
                Actions.runBlocking(experimentalCollectionPrep());
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose3.toPose2d(), "Cycle 4" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose3.toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( collectionPose.toPose2d() ); }
                Actions.runBlocking(experimentalExtend());
                sleep(275);
                Actions.runBlocking(collect());
                sleep(300);
                Actions.runBlocking(experimentalPreScore());
                sleep(100);
                if (logSpecimenSide) { specimenSideLog.logAction( collectionPose.toPose2d(), "Deposit 4" ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeTo(depositPose4.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose4.toPose2d() ); }
                Actions.runBlocking(extendScore());
                sleep(400);
                Actions.runBlocking(openClaw());
                sleep(300);
                Actions.runBlocking(stow());
                if (logSampleSide) { sampleSideLog.logAction( depositPose4.toPose2d(), "Park" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose4.toPose2d())
                                .strafeTo(parkPose.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( parkPose.toPose2d() ); }
            } else {

                //Experimental Rotational Auto
                if (logSpecimenSide) { specimenSideLog.logAction( startPose.toPose2d(), "Cycle 1" ); }
                Actions.runBlocking(experimentalPreScore());
                Actions.runBlocking(
                        drive.actionBuilder(startPose.toPose2d())
                                .strafeTo(depositPose.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose.toPose2d() ); }
                Actions.runBlocking(experimentalExtendScore2());
                sleep(200);
                Actions.runBlocking(floorPickUpPrep1());
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(anchorPose.toPose2d().position, Math.toRadians(45))
                                .build()
                );
                Actions.runBlocking(pickUp1());
                sleep(150);
                Actions.runBlocking(pickUp2());
                sleep(50);
                Actions.runBlocking(floorPickUp1());
                Actions.runBlocking(
                        drive.actionBuilder(anchorPose.toPose2d())
                                .turnTo(Math.toRadians(300))
                                .build()
                );
                Actions.runBlocking(openClaw());
                sleep(50);
                Actions.runBlocking(floorPickUpPrep1());
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2dWrapper(anchorPose.toPose2d().position.x, anchorPose.toPose2d().position.y, Math.toRadians(300)).toPose2d())
                                .strafeToLinearHeading(anchorPose2.toPose2d().position, Math.toRadians(45))
                                .build()
                );
                sleep(100);
                Actions.runBlocking(pickUp1());
                sleep(200);
                Actions.runBlocking(pickUp2());
                sleep(75);
                Actions.runBlocking(floorPickUp1());
                Actions.runBlocking(
                        drive.actionBuilder(anchorPose2.toPose2d())
                                .turnTo(Math.toRadians(300))
                                .build()
                );
                Actions.runBlocking(openClaw());
                sleep(50);
                Actions.runBlocking(floorPickUpPrep1());
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2dWrapper(anchorPose2.toPose2d().position.x, anchorPose2.toPose2d().position.y, Math.toRadians(300)).toPose2d())
                                .strafeToLinearHeading(anchorPose3.toPose2d().position, Math.toRadians(45))
                                .build()
                );
                Actions.runBlocking(pickUp1());
                sleep(100);
                Actions.runBlocking(pickUp2());
                sleep(50);
                Actions.runBlocking(floorPickUp1());
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2dWrapper(anchorPose3.toPose2d().position.x, anchorPose3.toPose2d().position.y, Math.toRadians(45)).toPose2d())
                                .strafeToLinearHeading(anchorPose4.toPose2d().position, anchorPose4.heading)
                                .build()
                );
                wristRotation.setPosition(0.39);
                Actions.runBlocking(experimentalCollectionPrep2());
                ///Cycle 3
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose2.toPose2d(), "Cycle 3" ); }
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2dWrapper(anchorPose4.x, anchorPose4.y, anchorPose4.heading).toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( collectionPose.toPose2d() ); }
                Actions.runBlocking(experimentalExtend());
                sleep(275);
                Actions.runBlocking(collect());
                sleep(250);
                Actions.runBlocking(experimentalPreScore());
                if (logSpecimenSide) { specimenSideLog.logAction( collectionPose.toPose2d(), "Deposit 3" ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeTo(depositPose2.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose3.toPose2d() ); }
                Actions.runBlocking(experimentalExtendScore());
                sleep(225);
                ///Cycle 4
                Actions.runBlocking(experimentalCollectionPrep());
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose3.toPose2d(), "Cycle 4" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose2.toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( collectionPose.toPose2d() ); }
                Actions.runBlocking(experimentalExtend());
                sleep(275);
                Actions.runBlocking(collect());
                sleep(250);
                Actions.runBlocking(experimentalPreScore());
                sleep(100);
                if (logSpecimenSide) { specimenSideLog.logAction( collectionPose.toPose2d(), "Deposit 4" ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeTo(depositPose3.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose4.toPose2d() ); }
                Actions.runBlocking(experimentalExtendScore());
                sleep(225);
                ///Cycle 4
                Actions.runBlocking(experimentalCollectionPrep());
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose3.toPose2d(), "Cycle 4" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose3.toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( collectionPose.toPose2d() ); }
                Actions.runBlocking(experimentalExtend());
                sleep(275);
                Actions.runBlocking(collect());
                sleep(250);
                Actions.runBlocking(experimentalPreScore());
                sleep(100);
                if (logSpecimenSide) { specimenSideLog.logAction( collectionPose.toPose2d(), "Deposit 4" ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeTo(depositPose4.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose4.toPose2d() ); }
                Actions.runBlocking(experimentalExtendScore());
                sleep(225);
                Actions.runBlocking(experimentalCollectionPrep());
                if (logSampleSide) { sampleSideLog.logAction( depositPose4.toPose2d(), "Park" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose4.toPose2d())
                                .strafeTo(collectionPose.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( collectionPose.toPose2d() ); }
                Actions.runBlocking(experimentalExtend());
                sleep(275);
                Actions.runBlocking(collect());
                sleep(250);
                Actions.runBlocking(experimentalPreScore());
                sleep(100);
                if (logSpecimenSide) { specimenSideLog.logAction( collectionPose.toPose2d(), "Deposit 4" ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeTo(depositPose5.toPose2d().position)
                                .build()
                );
                if (logSpecimenSide) { specimenSideLog.logAction( depositPose4.toPose2d() ); }
                Actions.runBlocking(experimentalExtendScore());
                sleep(225);
                Actions.runBlocking(stow());
                if (logSampleSide) { sampleSideLog.logAction( depositPose4.toPose2d(), "Park" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose5.toPose2d())
                                .strafeTo(parkPose.toPose2d().position)
                                .build()
                );
            }
        } else {
            if (experimental) {
                ///Cycle 1
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logAction( startPose.toPose2d(), "Cycle 1" ); }
                Actions.runBlocking(
                        drive.actionBuilder(startPose.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d() ); }
                sleep(500);
                Actions.runBlocking(openClaw());
                ///Cycle 2
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d(), "Cycle 2" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( collectionPose.toPose2d() ); }
                Actions.runBlocking(floorPickUpPrep());
                sleep(1500);
                Actions.runBlocking(pickUp());
                sleep(300);
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logAction( collectionPose.toPose2d(), "Deposit" ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(openClaw());
                sleep(500);
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d(), "Collection 2" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(collectionPose2.toPose2d().position, collectionPose2.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( collectionPose2.toPose2d() ); }
                Actions.runBlocking(floorPickUpPrep());
                sleep(1500);
                Actions.runBlocking(pickUp());
                sleep(500);
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logAction( collectionPose2.toPose2d(), "Deposit" ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose2.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(openClaw());
                sleep(300);
                Actions.runBlocking(lastFloorPickUpPrep());
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d(), "Collection 3" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(collectionPose3.toPose2d().position, collectionPose3.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( collectionPose3.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(lastFloorPickUp1());
                sleep(550);
                Actions.runBlocking(lastFloorPickUp2());
                sleep(500);
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logAction( collectionPose3.toPose2d(), "Deposit" ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose3.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(openClaw());
                sleep(500);
                Actions.runBlocking(armParkPose());
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d(), "Pre-Park" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(preParkPose.toPose2d().position, preParkPose.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( preParkPose.toPose2d() ); }
                if (logSampleSide) { sampleSideLog.logAction( preParkPose.toPose2d(), "Park" ); }
                Actions.runBlocking(
                        drive.actionBuilder(preParkPose.toPose2d())
                                .strafeToLinearHeading(parkPose.toPose2d().position, preParkPose.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( parkPose.toPose2d() ); }
                sleep(1500);
            } else {
                /// Auto code for when scoring Samples

                ///Cycle 1
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logAction( startPose.toPose2d(), "Cycle 1" ); }
                Actions.runBlocking(
                        drive.actionBuilder(startPose.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d() ); }
                sleep(500);
                Actions.runBlocking(openClaw());
                ///Cycle 2
                Actions.runBlocking(floorPickUpPrep());
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d(), "Sample 5" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(fifthSample.toPose2d().position, fifthSample.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( fifthSample.toPose2d() ); }
                sleep(500);
                Actions.runBlocking(pickUp());
                sleep(300);
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logAction( fifthSample.toPose2d(), "Deposit" ); }
                Actions.runBlocking(
                        drive.actionBuilder(fifthSample.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(openClaw());
                sleep(500);
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d(), "Collection" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( collectionPose.toPose2d() ); }
                Actions.runBlocking(floorPickUpPrep());
                sleep(700);
                Actions.runBlocking(pickUp());
                sleep(500);
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logAction( collectionPose.toPose2d(), "Deposit" ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(openClaw());
                sleep(300);
                Actions.runBlocking(floorPickUpPrep());
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d(), "Collection 2" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(collectionPose2.toPose2d().position, collectionPose2.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( collectionPose2.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(pickUp());
                sleep(500);
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logAction( collectionPose2.toPose2d(), "Deposit" ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose2.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(openClaw());
                sleep(300);
                Actions.runBlocking(lastFloorPickUpPrep());
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d(), "Collection 3" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(collectionPose3.toPose2d().position, collectionPose3.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( collectionPose3.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(lastFloorPickUp1());
                sleep(300);
                Actions.runBlocking(lastFloorPickUp2());
                sleep(500);
                Actions.runBlocking(highBucketPrep());
                if (logSampleSide) { sampleSideLog.logAction( collectionPose2.toPose2d(), "Deposit" ); }
                Actions.runBlocking(
                        drive.actionBuilder(collectionPose2.toPose2d())
                                .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d() ); }
                sleep(750);
                Actions.runBlocking(openClaw());
                sleep(500);
                Actions.runBlocking(armParkPose());
                if (logSampleSide) { sampleSideLog.logAction( depositPose.toPose2d(), "Pre-Park" ); }
                Actions.runBlocking(
                        drive.actionBuilder(depositPose.toPose2d())
                                .strafeToLinearHeading(preParkPose.toPose2d().position, preParkPose.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( preParkPose.toPose2d() ); }
                if (logSampleSide) { sampleSideLog.logAction( preParkPose.toPose2d(), "Park" ); }
                Actions.runBlocking(
                        drive.actionBuilder(preParkPose.toPose2d())
                                .strafeToLinearHeading(parkPose.toPose2d().position, preParkPose.toPose2d().heading)
                                .build()
                );
                if (logSampleSide) { sampleSideLog.logAction( parkPose.toPose2d() ); }
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
                wrist.setPosition(0.955);
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
                    clawServo.setPosition(0.2);
                    liftRotation.setTarget(760);
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
                clawServo.setPosition(0.3);
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
                liftRotation.setTarget(865);
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
                liftRotation.setTarget(865);
                liftExtension.setTarget(0);
                clawServo.setPosition(0.3);
                wrist.setPosition(0.5);
                return timer.milliseconds() > 1100;
            }
        };
    }

    public Action experimentalCollectionPrep2() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftRotation.setTarget(865);
                liftExtension.setTarget(0);
                clawServo.setPosition(0.3);
                wrist.setPosition(0.25);
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
                liftExtension.setTarget(480);
                return timer.milliseconds() >= 200;
            }
        };
    }

    public Action experimentalExtend() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftExtension.setTarget(385);
                wrist.setPosition(0.5);
                return timer.milliseconds() >= 220;
            }
        };
    }

    public Action experimentalExperimentalExtend() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftExtension.setTarget(860);
                return timer.milliseconds() >= 400;
            }
        };
    }

    public Action extendScore() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftExtension.setTarget(350);
                liftRotation.setTarget(2955);
                clawServo.setPosition(0.91);
                return timer.milliseconds() >= 300;
            }
        };
    }

    public Action experimentalExtendScore() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftExtension.setTarget(380);
                liftRotation.setTarget(2955);
                wrist.setPosition(0.3);
                return timer.milliseconds() >= 300;
            }
        };
    }

    public Action experimentalExtendScore2() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftExtension.setTarget(300);
                liftRotation.setTarget(2955);
                wrist.setPosition(0.3);
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
                wristRotation.setPosition(0.39);
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
                liftRotation.setTarget(650);
                liftExtension.setTarget(530);
                wrist.setPosition(0.2);
                return timer.milliseconds() > 500;
            }

        };
    }

    public Action floorPickUpPrep1() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftRotation.setTarget(670);
                liftExtension.setTarget(775);
                wrist.setPosition(0.2);
                wristRotation.setPosition(0.15);
                clawServo.setPosition(0.1);
                return timer.milliseconds() > 500;
            }

        };
    }

    public Action floorPickUp1() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftRotation.setTarget(730);
                liftExtension.setTarget(1080);
                wrist.setPosition(0.2);
                wristRotation.setPosition(0.39);
                return timer.milliseconds() > 500;
            }

        };
    }

    public Action floorPickUp2() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftRotation.setTarget(730);
                liftExtension.setTarget(970);
                wrist.setPosition(0.2);
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
                wristRotation.setPosition(0.56);
                return timer.milliseconds() > 500;
            }

        };
    }

    public Action lastFloorPickUp1() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftRotation.setTarget(445);
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
                liftRotation.getLiftMotor().setPower(0.75);
                clawServo.setPosition(0.95);
                liftRotation.setTarget(450);
                return timer.milliseconds() > 700;
            }
        };
    }

    public Action pickUp1() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftRotation.getLiftMotor().setPower(0.75);
                liftRotation.setTarget(450);
                wristRotation.setPosition(0.15);
                return timer.milliseconds() > 500;
            }
        };
    }

    public Action pickUp2() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                clawServo.setPosition(0.95);
                return timer.milliseconds() > 300;
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
