package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.LogFile;
import org.firstinspires.ftc.teamcode.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.math.maths.vectors.Vector3d;

@Config
@TeleOp()
public class MecanumTeleOpSandbox extends OpMode
{
    public static boolean logDetails = true;

    enum POSITION { SET_START, SET_END }
    enum LAST_ACTION { TBD, LT, RT, ITERATING }

    IMU imu;
    Orientation or;
    InputHandler inputHandler;
    Vector3d mecanumController;
    NewMecanumDrive drive;
    Action thisAction;

    ElapsedTime runtime = new ElapsedTime();
    LogFile detailsLog;
    double globalIMUHeading;
    double headingError = 0;
    boolean resetIMU = false;
    double[] dpadPowerArray = new double[4];
    double powerCoefficient = 1;
    boolean precisionDrive = false;
    double driveCoefficient = 1;
    Pose2d newStart = null;
    Pose2d newEnd = null;
    int iterations = 0;
    LAST_ACTION lastAction = LAST_ACTION.TBD;

    @Override
    public void init() {

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();

        if (logDetails) { detailsLog = new LogFile(LogFile.FileType.Details,"details", "csv"); }

        drive = new NewMecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), 0), detailsLog, logDetails);
        if (!drive.controlHub.isMacAddressValid()) {
            drive.controlHub.reportBadMacAddress(telemetry,hardwareMap);
            telemetry.update();
        }

        //Initialize gamepad handler
        inputHandler = InputAutoMapper.normal.autoMap(this);

        //values for gamepad joystick values represented as a vector3D
        mecanumController = new Vector3d();
    }

    @Override
    public void init_loop() {

        telemetry.addData("DPAD_UP: ", "increments 'iterations'");
        telemetry.addData("DPAD_DOWN: ", "decrements 'iterations'");
        telemetry.addData("B: ", "Move to target");
        telemetry.addData("X: ", "Move to origin");
        telemetry.addData("Y: ", "Moves robot 'iterations' times");

        if (inputHandler.up("D1:DPAD_UP")) {
            iterations++;
        }

        if (inputHandler.up("D1:DPAD_DOWN")) {
            iterations--;
            if (iterations<0) iterations = 0;
        }

        telemetry.addData("Compiled on:", BuildConfig.COMPILATION_DATE);
        telemetry.addData("Iterations:", iterations);
        telemetry.update();
        inputHandler.loop();
    }

    @Override
    public void start() {
        setPosition(POSITION.SET_START);
        runtime.reset();
    }

    @Override
    public void loop() {

        if (!handleInput()) stop();

        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        globalIMUHeading = or.thirdAngle;

        headingError = or.thirdAngle - globalIMUHeading;
        if(headingError > Math.PI) headingError -= 2*Math.PI;
        else if(headingError < -Math.PI) headingError += 2*Math.PI;

//        telemetry.addData("error: ", headingError);

        //Main Drive Update Code: :)
        resetIMU = drive.update(mecanumController, dpadPowerArray, headingError, resetIMU, powerCoefficient, precisionDrive);

        reportStartEnd();
    }

    private boolean handleInput() {
        inputHandler.loop();
//        mecanumController = new Vector3d((gamepad1.right_stick_x * driveCoefficient), (gamepad1.right_stick_y * driveCoefficient), (gamepad1.left_stick_x * driveCoefficient));
        mecanumController = new Vector3d((gamepad1.left_stick_x * driveCoefficient), (gamepad1.left_stick_y * driveCoefficient), (gamepad1.right_stick_x * driveCoefficient));

        if (inputHandler.up("D1:START")) {
            imu.resetYaw();
            telemetry.addLine("Reset Yaw");
            telemetry.update();
        }

        if (inputHandler.up("D1:Y")) {
            if (lastAction!=LAST_ACTION.ITERATING) {
                for (int i= 0 ; i<iterations ; i++) {
                    moveOut();
                    sleep(100);
                    moveBack();
                    sleep(100);
                }
            }
            lastAction = LAST_ACTION.ITERATING;   // Prevent robot from moving when it is already there
        }

        if (inputHandler.up("D1:LB")) {
            setPosition(POSITION.SET_START);
        } else if (inputHandler.up("D1:RB")) {
            setPosition(POSITION.SET_END);
        }

        if (inputHandler.up("D1:LT")) {
            if (lastAction!=LAST_ACTION.LT) moveBack();
            lastAction = LAST_ACTION.LT;   // Prevent robot from moving when it is already there
        } else if (inputHandler.up("D1:RT")) {
            if (lastAction!=LAST_ACTION.RT) moveOut();
            lastAction = LAST_ACTION.RT;   // Prevent robot from moving when it is already there
        }
        return true;
    }

    Pose2d getPosition() {
        drive.updatePoseEstimate();
        return new Pose2d( drive.pose.position.x, drive.pose.position.y, drive.pose.heading.toDouble() );
    }

    private void moveBack() {
        thisAction = drive.actionBuilder(newEnd)
                .strafeToConstantHeading(newStart.position)
                .build();
        Actions.runBlocking(thisAction);
    }

    private void moveOut() {
        thisAction = drive.actionBuilder(newStart)
                .strafeToConstantHeading(newEnd.position)
                .build();
        Actions.runBlocking(thisAction);
    }

    void reportStartEnd() {
        if (newStart!=null) {
            @SuppressLint("DefaultLocale")
            String start = "X: " + String.format("%.2f", newStart.position.x) + ", " +
                           "Y: " + String.format("%.2f", newStart.position.y) + ", " +
                           "Heading: " + String.format("%.2f", Math.toDegrees(newStart.heading.toDouble()));
            telemetry.addLine("Start - " + start);
        }
        if (newEnd!=null) {
            @SuppressLint("DefaultLocale")
            String end = "X: " + String.format("%.2f", newEnd.position.x) + ", " +
                         "Y: " + String.format("%.2f", newEnd.position.y) + ", " +
                         "Heading: " + String.format("%.2f", Math.toDegrees(newEnd.heading.toDouble()));
            telemetry.addLine("End   - " + end);
        }
        telemetry.update();
    }

    void setPosition( POSITION pos ) {
        if(pos==POSITION.SET_START) {
            newStart = getPosition();
        } else if(pos==POSITION.SET_END) {
            newEnd = getPosition();
        }
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}