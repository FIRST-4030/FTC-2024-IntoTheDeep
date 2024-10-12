package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.LinearMotorController;
import org.firstinspires.ftc.teamcode.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.math.maths.vectors.Vector3d;
import org.firstinspires.ftc.teamcode.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;

// aza test line to check commit
@TeleOp
public class MecanumTeleOp extends OpMode {

    final int LIFT_ROT_COEFFICIENT = 58;
    final int LIFT_EXT_COEFFICIENT = 30;
    NewMecanumDrive drive;
    InputHandler inputHandler;
    Vector3d mecanumController;
    LinearMotorController liftRotation, liftExtension;
    Servo clawServo;

    double driveCoefficient = 1;
    IMU imu;
    Orientation or;
    IMU.Parameters myIMUparameters;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime headingTimer = new ElapsedTime();
    double deltaTime;
    double previousTime;

    double globalIMUHeading;
    double headingError = 0;
    boolean resetIMU = false;

    //Create a hash map with keys: dpad buttons, and values: ints based on the corresponding joystick value of the dpad if is pressed and 0 if it is not
    //Ex. dpad Up = 1, dpad Down = -1
    //I chose to use a hashmap for human readability, even if it adds more lines of code, unsure if this was the correct choice but hey, I made it
    double[] dpadPowerArray = new double[4];
    double powerCoefficient = 1;
    boolean precisionDrive = false;
    boolean resetHeading = false;
    boolean clawOpen = true;

    double liftRotControl = 0;
    double liftExtControl = 0;


    DcMotor paralellEncoder;


    @Override
    public void init() {
        //init dpad hashmap with each dpad value as unpressed
        imu = hardwareMap.get(IMU.class, "imu");

        /*imu.initialize(new IMU.Parameters( new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));*/
        imu.resetYaw();
        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        globalIMUHeading = or.thirdAngle;


        //initialize drive, empty Vector as we are not using the Roadrunner drive methods in TeleOp
        drive = new NewMecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), 0));
        liftExtension = new LinearMotorController(hardwareMap, "slide",
                1390, false);
        liftRotation = new LinearMotorController(hardwareMap, "swing",
                9000, false);

        //Initialize gamepad handler
        inputHandler = InputAutoMapper.normal.autoMap(this);

        //values for gamepad joystick values represented as a vector3D
        mecanumController = new Vector3d();

        //Initialize encoder wheels
        //paralellEncoder = hardwareMap.get(DcMotor.class, "parallelEncoder");
        //paralellEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        handleInput();

        deltaTime = timer.milliseconds() - previousTime;
        previousTime += deltaTime;

        telemetry.addData("deltatime: ", deltaTime);
        telemetry.addData("Joystick X ", gamepad1.right_stick_x);
        telemetry.addData("Joystick Y ", gamepad1.right_stick_y);
        telemetry.addData("Joystick Z ", gamepad1.left_stick_x);

        telemetry.addData("par: ", drive.rightBack.getCurrentPosition());
        telemetry.addData("perp ", drive.rightFront.getCurrentPosition());



        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        headingError = or.thirdAngle - globalIMUHeading;
        telemetry.addData("error: ", headingError);

        //Main Drive Update Code: :)
        resetIMU = drive.update(mecanumController, dpadPowerArray, headingError, resetIMU, powerCoefficient, precisionDrive);
        liftExtension.update(liftExtControl, LIFT_EXT_COEFFICIENT);
        liftRotation.update(liftRotControl, LIFT_ROT_COEFFICIENT);
        telemetry.addData("Rot: ", liftRotation.target);
        telemetry.addData("Ext: ", liftExtension.target);

        telemetry.update();
        outputLog();
    }
    public void handleInput() {
        inputHandler.loop();
        mecanumController = new Vector3d((gamepad1.right_stick_x * driveCoefficient), (gamepad1.right_stick_y * driveCoefficient), (gamepad1.left_stick_x * driveCoefficient));
        liftRotControl = gamepad2.left_stick_y;
        liftExtControl = gamepad2.right_stick_y;
        //Reset Field-Centric drive by pressing B
        if(inputHandler.up("D1:B")){
            resetIMU = true;
        }

        if(gamepad1.left_stick_x != 0){
            resetHeading = true;
            headingTimer.reset();
        }

        if(resetHeading){
            if(headingTimer.milliseconds() > 250){
                globalIMUHeading = or.thirdAngle;
                resetHeading = false;
            }
        }
        if(inputHandler.up("D2:X")){

        }

        if(inputHandler.up("D2:Y")){
            liftRotation.setTarget(9000);
        }


    }
    public void outputLog() {

    }
}








