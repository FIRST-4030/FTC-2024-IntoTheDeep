package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.LinearMotorController;
import org.firstinspires.ftc.teamcode.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.teamcode.ComputerVision;

@Config
@Autonomous(name = "MecanumAuto")
public class MecanumAuto extends LinearOpMode {


    ComputerVision vision;
    AprilTagPoseFtc[] aprilTagTranslations = new AprilTagPoseFtc[11];
    InputHandler inputHandler;
    LinearMotorController liftRotation;
    LinearMotorController liftExtension;
    Servo clawServo, wrist;
    boolean liftNotAtPosition = true;

    boolean inputComplete = false;
    boolean side = true;
    Pose2d robotPose;

    ElapsedTime runtime = new ElapsedTime();
    int startDelay = 0;
    int i = 1; //used as an iterator for outputLog()

    public Pose2dWrapper startPose;
    public Pose2dWrapper depositPose;
    public Pose2dWrapper pushPrep1;
    public Pose2dWrapper pushPrep2;
    public Pose2dWrapper pushPrep3;
    public Pose2dWrapper brickPush1;
    public Pose2dWrapper collectionPose;
    public Pose2dWrapper intermediaryPose;
    public Pose2dWrapper depositPose2;
    public Pose2dWrapper parkPose;
    public Pose2dWrapper depositPose3;
    public Pose2dWrapper depositPose4;

    public Pose2dWrapper collectionPose2;
    public Pose2dWrapper collectionPose3;




    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();
        liftExtension = new LinearMotorController(hardwareMap, "slide",
                1390, false);
        liftRotation = new LinearMotorController(hardwareMap, "swing",
                3000, false);
        wrist = hardwareMap.get(Servo.class, "wrist");
        clawServo = hardwareMap.get(Servo.class, "claw");
        inputHandler = InputAutoMapper.normal.autoMap(this);
        clawServo.setPosition(0.95);
        while (!inputComplete) {
            inputHandler.loop();

            if (inputHandler.up("D1:X")) {
                inputComplete = true;
            }
            if(inputHandler.up("D1:RT")){
                side = !side;
            }
            telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
            telemetry.addData("-----Initialization-----", "");
            telemetry.addLine();
            telemetry.addData("Auto Mode: ", side ? "Sample Scoring" : "Specimen Scoring");
            //telemetry.addData("-----Modifications-----", "");
            //telemetry.addLine();
            telemetry.addData("Press X to finalize values", inputComplete);
            telemetry.update();

        }
        if(!side) {
            ///Positions for when scoring on Specimen Side
            startPose = new Pose2dWrapper(15.8, -55.75, 1.5708);
            depositPose = new Pose2dWrapper(15.8, -27.5, 1.5708);
            pushPrep1 = new Pose2dWrapper(40.8, -30.75, 1.5708);
            pushPrep2 = new Pose2dWrapper(39.8, -7.75, 1.5708);
            pushPrep3 = new Pose2dWrapper(51.8, -7.75, 1.5708);
            brickPush1 = new Pose2dWrapper(51.8, -51.75, 1.5708);
            collectionPose = new Pose2dWrapper(46.5, -66.5, 0);
            intermediaryPose = new Pose2dWrapper(43, -63.5, 1.5708);
            depositPose2 = new Pose2dWrapper(9.8, -27.5, 1.5708);
            parkPose = new Pose2dWrapper(51.8, -51.75, 1.5708);
            depositPose3 = new Pose2dWrapper(4.8, -27.5, 1.5708);
            depositPose4 = new Pose2dWrapper(0, -27.5, 1.5708);
        } else {
            ///Positions for when scoring on Sample Side
            startPose = new Pose2dWrapper(-32, -56, 1.5708);
            depositPose = new Pose2dWrapper(-63, -46, 1.5708);
            collectionPose = new Pose2dWrapper(-50, -35, 1.5708);
            collectionPose2 = new Pose2dWrapper(-58.5, -36, 1.5708);
            collectionPose3 = new Pose2dWrapper(-60, -21.5, 0.785+1.5708);
            parkPose = new Pose2dWrapper(-22, -7, 3.1415);
        }
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap, startPose.toPose2d());


        ///START AUTO:
        waitForStart();
        if (isStopRequested()) return;
        telemetry.addData("Started Running", " ");
        telemetry.update();
        outputLog(drive);
        liftExtension.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRotation.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftExtension.getLiftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRotation.getLiftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /// Auto code for when scoring Specimens
        if(!side) {
            Actions.runBlocking(
                    preScore()
            );
            Actions.runBlocking(
                    drive.actionBuilder(startPose.toPose2d())
                            .strafeTo(depositPose.toPose2d().position)
                            .build()
            );
            Actions.runBlocking(
                    armDown()
            );
            sleep(500);
            Actions.runBlocking(
                    openClaw()
            );

            sleep(500);
            Actions.runBlocking(
                    stow()
            );
            sleep(500);
            Actions.runBlocking(
                    drive.actionBuilder(depositPose.toPose2d())
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
                            .strafeTo(brickPush1.toPose2d().position)
                            .build()
            );

            Actions.runBlocking(
                    collectionPrep2()
            );

            Actions.runBlocking(
                    drive.actionBuilder(brickPush1.toPose2d())
                            .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                            .build()
            );

            sleep(400);


            Actions.runBlocking(
                    extend()
            );

            sleep(300);

            Actions.runBlocking(
                    collect()
            );


            sleep(300);

            Actions.runBlocking(
                    preScore()
            );

            sleep(500);

            Actions.runBlocking(
                    drive.actionBuilder(collectionPose.toPose2d())
                            .strafeToLinearHeading(intermediaryPose.toPose2d().position, intermediaryPose.heading)
                            .build()
            );

            Actions.runBlocking(
                    drive.actionBuilder(intermediaryPose.toPose2d())
                            .strafeTo(depositPose2.toPose2d().position)
                            .build()
            );

            sleep(500);

            Actions.runBlocking(
                    armDown()
            );

            sleep(500);

            Actions.runBlocking(
                    openClaw()
            );
            sleep(500);

            Actions.runBlocking(
                    resetArm()
            );

            sleep(500);

            Actions.runBlocking(
                    collectionPrep()
            );

            Actions.runBlocking(
                    drive.actionBuilder(depositPose2.toPose2d())
                            .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                            .build()
            );

            sleep(500);


            Actions.runBlocking(
                    extend()
            );

            sleep(500);

            Actions.runBlocking(
                    collect()
            );

            sleep(500);

            Actions.runBlocking(
                    preScore()
            );

            Actions.runBlocking(
                    drive.actionBuilder(collectionPose.toPose2d())
                            .strafeToLinearHeading(intermediaryPose.toPose2d().position, intermediaryPose.heading)
                            .build()
            );

            Actions.runBlocking(
                    drive.actionBuilder(intermediaryPose.toPose2d())
                            .strafeTo(depositPose3.toPose2d().position)
                            .build()
            );

            sleep(500);
            Actions.runBlocking(
                    armDown()
            );

            sleep(500);

            Actions.runBlocking(
                    openClaw()
            );
            sleep(500);

            Actions.runBlocking(
                    resetArm()
            );

            Actions.runBlocking(
                    drive.actionBuilder(depositPose3.toPose2d())
                            .strafeToLinearHeading(collectionPose.toPose2d().position, collectionPose.heading)
                            .build()
            );
            sleep(500);

            Actions.runBlocking(
                    collectionPrep()
            );

            Actions.runBlocking(
                    extend()
            );

            sleep(500);

            Actions.runBlocking(
                    collect()
            );

            sleep(500);
            Actions.runBlocking(
                    preScore()
            );
            sleep(500);
            Actions.runBlocking(
                    drive.actionBuilder(collectionPose.toPose2d())
                            .strafeToLinearHeading(intermediaryPose.toPose2d().position, intermediaryPose.heading)
                            .build()
            );
            sleep(500);
            Actions.runBlocking(
                    drive.actionBuilder(intermediaryPose.toPose2d())
                            .strafeTo(depositPose4.toPose2d().position)
                            .build()
            );
            Actions.runBlocking(
                    armDown()
            );
            sleep(500);

            Actions.runBlocking(
                    openClaw()
            );
            sleep(500);

            Actions.runBlocking(
                    resetArm()
            );
            Actions.runBlocking(
                    drive.actionBuilder(depositPose4.toPose2d())
                            .strafeTo(parkPose.toPose2d().position)
                            .build()
            );


        } else {
            /// Auto code for when scoring Samples
            Actions.runBlocking(
                    highBucketPrep()
            );

            Actions.runBlocking(
                    drive.actionBuilder(startPose.toPose2d())
                            .strafeTo(depositPose.toPose2d().position)
                            .build()
            );

            telemetry.addData("start","");
            telemetry.update();
            sleep(300);

            Actions.runBlocking(
                    openClaw()
            );
            sleep(500);
            telemetry.addData("openClaw","");
            telemetry.update();
            sleep(300);
            Actions.runBlocking(
                    floorPickUpPrep()
            );
            telemetry.addData("floorPickUpPrep","");
            telemetry.update();
            Actions.runBlocking(
                    drive.actionBuilder(depositPose.toPose2d())
                            .strafeTo(collectionPose.toPose2d().position)
                            .build()
            );
            telemetry.addData("move","");
            telemetry.update();
            sleep(300);
            Actions.runBlocking(
                    pickUp()
            );
            sleep(300);
            Actions.runBlocking(
                    highBucketPrep()
            );


            Actions.runBlocking(
                    drive.actionBuilder(collectionPose.toPose2d())
                            .strafeTo(depositPose.toPose2d().position)
                            .build()
            );
            sleep(500);

            Actions.runBlocking(
                    openClaw()
            );
            sleep(500);
            Actions.runBlocking(
                    floorPickUpPrep()
            );
            sleep(500);
            Actions.runBlocking(
                    drive.actionBuilder(depositPose.toPose2d())
                            .strafeTo(collectionPose2.toPose2d().position)
                            .build()
            );
            sleep(300);
            Actions.runBlocking(
                    pickUp()
            );
            sleep(500);
            Actions.runBlocking(
                    highBucketPrep()
            );

            Actions.runBlocking(
                    drive.actionBuilder(collectionPose2.toPose2d())
                            .strafeTo(depositPose.toPose2d().position)
                            .build()
            );
            sleep(500);
            Actions.runBlocking(
                    openClaw()
            );
            sleep(500);
            Actions.runBlocking(
                    floorPickUpPrep()
            );

            Actions.runBlocking(
                    drive.actionBuilder(depositPose.toPose2d())
                            .strafeToLinearHeading(collectionPose3.toPose2d().position, collectionPose3.toPose2d().heading)
                            .build()
            );
            sleep(500);
            Actions.runBlocking(
                    pickUp()
            );
            sleep(500);
            Actions.runBlocking(
                    highBucketPrep()
            );

            Actions.runBlocking(
                    drive.actionBuilder(collectionPose3.toPose2d())
                            .strafeToLinearHeading(depositPose.toPose2d().position, depositPose.toPose2d().heading)
                            .build()
            );

            Actions.runBlocking(
                    openClaw()
            );
            sleep(500);
            Actions.runBlocking(
                    stow()
            );

            /*
            Actions.runBlocking(
                    drive.actionBuilder(depositPose.toPose2d())
                            .strafeToLinearHeading(parkPose.toPose2d().position, parkPose.toPose2d().heading)
                            .build()
            );
            */
            Actions.runBlocking(
                    armParkPose()
            );




            }

    }

    public Action preScore() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                liftRotation.setTarget(2000);
                liftExtension.setTarget(150);
                wrist.setPosition(0.4);
                clawServo.setPosition(0.95);
                return liftRotation.getLiftMotor().getCurrentPosition() > 1960;
            }
        };
    }

    public Action armDown() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                    liftRotation.setTarget(1332);
                    return liftRotation.getLiftMotor().getCurrentPosition() <= 1350;
            }
        };
    }

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

    public Action pickUp() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                clawServo.setPosition(0.95);
                liftRotation.setTarget(500);
                return timer.milliseconds() > 500;
            }
        };
    }

    public Action stow() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftExtension.setTarget(0);
                liftRotation.setTarget(10);
                wrist.setPosition(1);
                return liftRotation.getLiftMotor().getCurrentPosition() <= 250;
            }
        };
    }

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

    public Action collectionPrep2() {
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

    public Action extend() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftExtension.setTarget(275);
                return timer.milliseconds() >= 200;
            }
        };
    }


    public Action collect() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                clawServo.setPosition(0.95);
                return timer.milliseconds() > 300;
            }
        };
    }

    public Action resetArm() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftExtension.setTarget(0);
                liftRotation.setTarget(0);
                clawServo.setPosition(0.95);
                wrist.setPosition(1);
                return timer.milliseconds() > 500;
            }
        };
    }

    public Action highBucketPrep() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                wrist.setPosition(0.775);
                clawServo.setPosition(0.95);
                liftRotation.setTarget(3000);
                liftExtension.setTarget(1390);
                return timer.milliseconds() > 1200;
            }
        };
    }

    public Action floorPickUpPrep() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ElapsedTime timer = new ElapsedTime();
                liftRotation.setTarget(500);
                liftExtension.setTarget(530);
                wrist.setPosition(0.15);
                return timer.milliseconds() > 500;
            }

        };
    }

    public Action armParkPose() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftRotation.setTarget(1500);
                liftExtension.setTarget(300);
                wrist.setPosition(0.90);
                return liftRotation.getLiftMotor().getCurrentPosition() >= 1540;
            }
        };
    }



    public void outputLog (NewMecanumDrive drive){
        RobotLog.d("WAY: Current Robot Pose Estimate and time: X: %.03f Y: %.03f Heading: %.03f ms: %.03f iteration: %d",
                drive.pose.position.x,
                drive.pose.position.y,
                Math.toDegrees(drive.pose.heading.real),
                runtime.milliseconds(), i);
        i++;
    }
}
