package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

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
    Pose2d robotPose;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime inputTimer = new ElapsedTime();
    int startDelay = 0;
    int i = 1; //used as an iterator for outputLog()

    public Pose2dWrapper startPose = new Pose2dWrapper(15.8, -55.75, 1.5708);
    public Pose2dWrapper depositPose = new Pose2dWrapper(15.8, -32.75, 1.5708);
    public Pose2dWrapper pushPrep1 = new Pose2dWrapper(39.8, -32.75, 1.5708);
    public Pose2dWrapper pushPrep2 = new Pose2dWrapper(39.8, -8.75, 1.5708);
    public Pose2dWrapper pushPrep3 = new Pose2dWrapper(51.8, -8.75, 1.5708);
    public Pose2dWrapper brickPush1 = new Pose2dWrapper(51.8, -51.75, 1.5708);
    public Pose2dWrapper pushPrep4 = new Pose2dWrapper(56.8, -8.75, 1.5708);
    public Pose2dWrapper brickPush2 = new Pose2dWrapper(56.8, -51.75, 1.5708);
    public Pose2dWrapper pushPrep5 = new Pose2dWrapper(61.8, -8.75, 1.5708);
    public Pose2dWrapper brickPush3 = new Pose2dWrapper(61.8, -51.75, 1.5708);


    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();
        liftExtension = new LinearMotorController(hardwareMap, "slide",
                1390, false);
        liftRotation = new LinearMotorController(hardwareMap, "swing",
                4500, false);
        wrist = hardwareMap.get(Servo.class, "wrist");
        clawServo = hardwareMap.get(Servo.class, "claw");
        inputHandler = InputAutoMapper.normal.autoMap(this);
        clawServo.setPosition(0.95);
        /*while (inputComplete == false) {


            if (inputHandler.up("D1:X")) {
                inputComplete = true;
                inputTimer.reset();
            }
            telemetry.addData("-----Initialization-----", "");
            telemetry.addLine();
            telemetry.addData("-----Modifications-----", "");
            telemetry.addLine();
            telemetry.addData("Press X to finalize values", inputComplete);
            telemetry.update();

        }


        vision = new ComputerVision(hardwareMap);
        while (vision.visionPortal.getCameraState() == OPENING_CAMERA_DEVICE) {}

        vision.setActiveCameraOne();*/
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap, startPose.toPose2d());

        waitForStart();

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

        Actions.runBlocking(
                openClaw()
        );

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
                drive.actionBuilder(brickPush1.toPose2d())
                        .strafeTo(pushPrep4.toPose2d().position)
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(pushPrep4.toPose2d())
                        .strafeTo(brickPush2.toPose2d().position)
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(brickPush2.toPose2d())
                        .strafeTo(pushPrep5.toPose2d().position)
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(pushPrep5.toPose2d())
                        .strafeTo(brickPush3.toPose2d().position)
                        .build()
        );





        if (isStopRequested()) return;
        telemetry.addData("Started Running", " ");
        telemetry.update();
        sleep(startDelay);
        outputLog(drive); //1

    }

    public Action preScore() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                liftRotation.setTarget(3000);
                liftExtension.setTarget(150);
                wrist.setPosition(0.4);
                clawServo.setPosition(0.95);
                return liftRotation.getLiftMotor().getCurrentPosition() > 2950;
            }
        };
    }

    public Action armDown() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                    liftRotation.setTarget(2000);
                    return liftRotation.getLiftMotor().getCurrentPosition() <= 2050;
            }
        };
    }

    public Action openClaw() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                    ElapsedTime timer = new ElapsedTime();
                    clawServo.setPosition(0.3);
                    return timer.milliseconds() < 75;
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
