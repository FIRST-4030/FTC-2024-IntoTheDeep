package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.LogFile;
import org.firstinspires.ftc.teamcode.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;
import org.firstinspires.ftc.teamcode.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.math.maths.vectors.Vector3d;

@Config
@TeleOp()
public class MecanumTeleOpSandbox extends OpMode
{
    public static double startX = 8;
    public static double startY = -12;
    public static double startHeading = 90;
    public static double stabilizerX = -15;
    public static double stabilizerY = 12;
    public static double stabilizerHeading = 270;

    public static boolean logDetails = false;

    boolean inputComplete = false;
    IMU imu;
    InputHandler inputHandler;
    Vector3d mecanumController;
    NewMecanumDrive drive;

    ElapsedTime inputTimer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();
    LogFile detailsLog;

    public static Pose2dWrapper startPose = new Pose2dWrapper(startX, startY, startHeading);
    public static Pose2dWrapper stabilizerPose = new Pose2dWrapper(stabilizerX, stabilizerY, stabilizerHeading);

    @Override
    public void init() {

        imu = hardwareMap.get(IMU.class, "imu");
        runtime.reset();

        drive = new NewMecanumDrive(hardwareMap, startPose.toPose2d(), detailsLog, logDetails);

        //Initialize gamepad handler
        inputHandler = InputAutoMapper.normal.autoMap(this);

        //values for gamepad joystick values represented as a vector3D
        mecanumController = new Vector3d();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
//        telemetry.addLine();
//        if (inputTimer.milliseconds() > 250) {
//
//            if (inputHandler.up("D1:X")) {
//                inputComplete = true;
//                inputTimer.reset();
//            }
//        }
//
//        telemetry.addData("Press X to finalize values", inputComplete);
        telemetry.update();
        inputHandler.loop();
    }

    @Override
    public void start() { runtime.reset(); }

    @Override
    public void loop() {
        handleInput();
    }

    @Override
    public void stop() {
        telemetry.addData("Job completed!", "");
        telemetry.update();
    }

    void handleInput() {
        inputHandler.loop();

        if (inputHandler.up("D1:START")) {
            imu.resetYaw();
            telemetry.addData("Reset Yaw", "");
            telemetry.update();
            inputTimer.reset();
        }

        if (inputHandler.up("D1:A")) {
            moveFromWallToSubmersible();
            inputTimer.reset();
        }

        if (inputHandler.up("D1:B")) {
            moveFromSubmersibleToWall();
            inputTimer.reset();
        }

        telemetry.update();
    }
        void moveFromWallToSubmersible() {
        Action stabilizerAction = drive.actionBuilder(startPose.toPose2d())
                    .strafeToLinearHeading(stabilizerPose.toPose2d().position,stabilizerPose.toPose2d().heading)
                    .build();
        Actions.runBlocking(stabilizerAction);
    }

    void moveFromSubmersibleToWall() {
        Action startAction = drive.actionBuilder(stabilizerPose.toPose2d())
                    .strafeToLinearHeading(startPose.toPose2d().position,startPose.toPose2d().heading)
                    .build();
        Actions.runBlocking(startAction);
    }
}