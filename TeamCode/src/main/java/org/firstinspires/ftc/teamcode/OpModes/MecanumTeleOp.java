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
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.8;     // Maximum rotational position
    static final double MIN_POS     =  0.3;     // Minimum rotational position

    // Define class members
    double  position = ((MAX_POS - MIN_POS) / 2)+0.01; // Start at halfway position
    boolean rampUp = true;

    double driveCoefficient = 1;
    IMU imu;
    Orientation or;
    IMU.Parameters myIMUparameters;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime headingTimer = new ElapsedTime();
    ElapsedTime clawTimer;
    double deltaTime;
    double previousTime;
    boolean liftNotAtPosition = true;
    boolean scoreSpecimen = true;

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
    double yawToFix;
    double cleanYaw;
    double priorYaw = 0;
    double deltaYaw;
    double savedOrientation;
    boolean quickRot = false;


    DcMotor paralellEncoder;


    @Override
    public void init() {
        //init dpad hashmap with each dpad value as unpressed
        imu = hardwareMap.get(IMU.class, "imu");
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawServo.setPosition(0.3);


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




        clawTimer = new ElapsedTime();

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

        if (gamepad2.a && clawTimer.seconds() > 0.5) {
            clawTimer.reset();
            if (rampUp) {
                position = MAX_POS;
                rampUp = !rampUp;   // Switch ramp direction
            }
            else {
                position = MIN_POS;
                rampUp = !rampUp;  // Switch ramp direction
            }
        }

        // Display the current value
        telemetry.addData("Servo Position", "%5.2f", position);
        telemetry.addData("time", clawTimer.seconds());
        telemetry.addData(">", "Press Stop to end test." );
        telemetry.update();

        telemetry.addData("deltatime: ", deltaTime);
        telemetry.addData("Joystick X ", gamepad1.right_stick_x);
        telemetry.addData("Joystick Y ", gamepad1.right_stick_y);
        telemetry.addData("Joystick Z ", gamepad1.left_stick_x);

        telemetry.addData("par: ", drive.rightBack.getCurrentPosition());
        telemetry.addData("perp ", drive.rightFront.getCurrentPosition());



        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

        headingError = or.thirdAngle - globalIMUHeading;
        if(headingError > Math.PI) deltaYaw -= 2*Math.PI;
        else if(headingError < -Math.PI) deltaYaw +=Math.PI;


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

        if(inputHandler.up("D2:Y")){
            scoreSpecimen = true;
            liftNotAtPosition = true;
        }

        if(scoreSpecimen){ postSpecimenScoringPos();}

        if(resetHeading){
            if(headingTimer.milliseconds() > 250){
                globalIMUHeading = or.thirdAngle;
                resetHeading = false;
            }
        }
        if(inputHandler.up("D2:X")){
            if(clawOpen){
                clawServo.setPosition(0);
                clawOpen = !clawOpen;
            } else {
                clawServo.setPosition(0.3);
                clawOpen = !clawOpen;
            }
        }

        if(inputHandler.up("D2:Y")){
            specimenCollectionPos();
        }

        if(inputHandler.up("D1:RB")){
            globalIMUHeading = or.thirdAngle + Math.PI/2;
        }





    }

    public void specimenCollectionPos(){
        liftRotation.setTarget(1330);
        liftExtension.setTarget(500);
        clawServo.setPosition(0.3);
    }
    public void preSpecimenScoringPos(){
        liftRotation.setTarget(3000);
        liftExtension.setTarget(1020);
    }
    public void postSpecimenScoringPos() {
        if(liftNotAtPosition){
        liftRotation.setTarget(2900);
        }
        if(liftRotation.getLiftMotor().getCurrentPosition() <= 2900)
        {
            clawServo.setPosition(0.3);
            liftNotAtPosition = false;
            scoreSpecimen = false;
        }

    }

    public void outputLog() {

    }
}