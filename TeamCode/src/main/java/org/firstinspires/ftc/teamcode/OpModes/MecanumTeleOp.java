package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

//TODO: Make the controller more sensitive
@TeleOp
public class MecanumTeleOp extends OpMode {

    //0.00370370 = 1 degree on a 270 servo
    final int LIFT_ROT_COEFFICIENT = 39;
    final int LIFT_EXT_COEFFICIENT = 30;
    final double STARTING_LENGTH = 14.5;
    final double SERVO_DELTA_PER_DEGREE_270 = 0.00370370370;
    final int MINIMUM_DEGREES = 55;
    NewMecanumDrive drive;
    InputHandler inputHandler;
    Vector3d mecanumController;
    LinearMotorController liftRotation, liftExtension;
    Servo clawServo, wrist;
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.8;     // Maximum rotational position
    static final double MIN_POS     =  0.3; // Minimum rotational position
    boolean slowMode = false;
    boolean trig = false;

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
    boolean scoreSpecimen = false;

    double globalIMUHeading;
    double headingError = 0;
    boolean resetIMU = false;
    boolean hanging = false;
    boolean basket = false;

    //TRIG BELOW:

    double theta;
    static final double HIGH_TARGET = 2.5;
    static final double LOW_TARGET = 5.25;
    double controlledTarget = 2.5;
    double extensionInches = STARTING_LENGTH;
    double extensionTicksPerIn = 75.13;
    double rotationTicksPerDegree = 25;
    boolean highTarget = true;
    boolean lowTarget = false;
    boolean driverControlled = false;

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


    DcMotor paralellEncoder;


    @Override
    public void init() {
        //init dpad hashmap with each dpad value as unpressed
        imu = hardwareMap.get(IMU.class, "imu");
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawServo.setPosition(0.95);
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(1);

        /*
        imu.initialize(new IMU.Parameters( new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));*/
        imu.resetYaw();
        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        globalIMUHeading = or.thirdAngle;

        //initialize drive, empty Vector as we are not using the Roadrunner drive methods in TeleOp
        drive = new NewMecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), 0));
        liftExtension = new LinearMotorController(hardwareMap, "slide",
                1390, false);
        liftRotation = new LinearMotorController(hardwareMap, "swing",
                3000, false);




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
        if(slowMode || liftExtension.getLiftMotor().getCurrentPosition() > 750) {
            driveCoefficient = 0.65;
        } else {
            driveCoefficient = 1;
        }
        handleInput();

        deltaTime = timer.milliseconds() - previousTime;
        previousTime += deltaTime;


        // Display the current value
        telemetry.addData("Yaw: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Pitch: ", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));
        telemetry.addData("Roll: ", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS));



        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

        headingError = or.thirdAngle - globalIMUHeading;
        if(headingError > Math.PI) headingError -= 2*Math.PI;
        else if(headingError < -Math.PI) headingError += 2*Math.PI;


        telemetry.addData("error: ", headingError);

        //Main Drive Update Code: :)
        resetIMU = drive.update(mecanumController, dpadPowerArray, headingError, resetIMU, powerCoefficient, precisionDrive);
        if(trig) {
            liftExtension.update(liftExtControl, LIFT_EXT_COEFFICIENT);
            extensionInches = (liftExtension.getLiftMotor().getCurrentPosition() / extensionTicksPerIn)
                    + STARTING_LENGTH;
            if(driverControlled){
                theta = 180/Math.PI * (Math.acos(controlledTarget / extensionInches));
            }else if(highTarget) {
                theta = 180/Math.PI * (Math.acos(HIGH_TARGET / extensionInches));
            } else if (lowTarget){
                theta = 180/Math.PI * (Math.acos(LOW_TARGET / extensionInches));
            }
            telemetry.addData("Trig Theta Angle: ", theta);
            liftRotation.setTarget((int) (((theta * rotationTicksPerDegree) - (MINIMUM_DEGREES * rotationTicksPerDegree))));
            wrist.setPosition(0.5 - theta * SERVO_DELTA_PER_DEGREE_270);

            //liftExtension.update(liftExtControl, LIFT_EXT_COEFFICIENT);
            //liftRotation.update(liftRotControl, LIFT_ROT_COEFFICIENT);

        } else {
            liftExtension.update(liftExtControl, LIFT_EXT_COEFFICIENT);
            liftRotation.update(liftRotControl, LIFT_ROT_COEFFICIENT);
        }

        if(basket) {
            liftExtension.setTickLimit(1390);
        } else {
            liftExtension.setTickLimit(1310);
        }

        telemetry.addData("Rot: ", liftRotation.target);
        telemetry.addData("Ext: ", liftExtension.target);

        if(liftExtension.getLiftMotor().getCurrentPosition() > 925
                && wrist.getPosition() < 0.6
                && wrist.getPosition() > 0.4) {
            wrist.setPosition(0.05);
        }

        telemetry.update();
        outputLog();
    }
    public void handleInput() {
        inputHandler.loop();
        mecanumController = new Vector3d((gamepad1.right_stick_x * driveCoefficient), (gamepad1.right_stick_y * driveCoefficient), (gamepad1.left_stick_x * driveCoefficient));

        liftRotControl = gamepad2.left_stick_y;

        if(inputHandler.active("D2:DPAD_UP")){
            driverControlled = true;
            if(controlledTarget > 2) {
                controlledTarget -= 0.06;
            }
        }

        if(inputHandler.active("D2:DPAD_DOWN")){
            driverControlled = true;
            if(controlledTarget < 5.25) {
                controlledTarget += 0.06;
            }

        }

        if(!hanging) {
            liftExtControl = gamepad2.right_stick_y;
        }

        //Reset Field-Centric drive by pressing B
        if(inputHandler.up("D1:B")){
            resetIMU = true;
        }

        slowMode = inputHandler.active("D1:RT");

        if(gamepad1.left_stick_x != 0){
            resetHeading = true;
            headingTimer.reset();
        }

        if(inputHandler.up("D2:Y")){
            scoreSpecimen = true;
            liftNotAtPosition = true;
        }

        if(inputHandler.up("D2:LT")){
            sampleCollectionPos();
        }

        if(inputHandler.up("D2:LB")){
            driverControlled = false;
            highTarget = !highTarget;
            lowTarget = !lowTarget;
        }

        if(scoreSpecimen){
            postSpecimenScoringPos();
        }

        if(resetHeading){
            if(headingTimer.milliseconds() > 250){
                globalIMUHeading = or.thirdAngle;
                resetHeading = false;
            }
        }
        if(inputHandler.up("D2:X")){
            if(!clawOpen){
                clawServo.setPosition(0.3);
                clawOpen = !clawOpen;
            } else {
                clawServo.setPosition(0.95);
                clawOpen = !clawOpen;
            }
        }

        if(inputHandler.up("D2:A")){
            specimenCollectionPos();
        }
        if(inputHandler.up("D2:RT")){
            preSpecimenScoringPos();
        }

        if(inputHandler.up("D1:RB")){
            globalIMUHeading = or.thirdAngle - Math.PI/2;
        }

        if(inputHandler.up("D1:LB")){
            globalIMUHeading = or.thirdAngle + Math.PI/2;
        }

        if(inputHandler.up("D2:RB")){
            highBasket();
        }
        if(inputHandler.up("D1:LT")){
            beginHang();
        }





    }

    public void beginHang(){
        trig = false;
        basket = false;
        if(hanging){
            hanging = false;
            liftRotation.setTickLimit(3000);
            liftRotation.setTarget(3000);
        } else {
            hanging = true;
            liftRotation.setTickLimit(3333);
            liftRotation.setTarget(3333);
            liftExtension.setTarget(5);
            clawServo.setPosition(0.1);
            wrist.setPosition(0.01);

        }
    }

    public void specimenCollectionPos(){
        trig = false;
        basket = false;
        liftRotation.setTarget(886);
        liftExtension.setTarget(500);
        clawServo.setPosition(0.95);
        clawOpen = false;
        wrist.setPosition(0.5);
    }

    public void highBasket() {
        trig = false;
        basket = true;
        wrist.setPosition(0.775);
        clawServo.setPosition(0.95);
        clawOpen = false;

        liftRotation.setTarget(3000);
        liftExtension.setTarget(1390);
    }

    public void preSpecimenScoringPos(){
        trig = false;
        basket = false;
        liftRotation.setTarget(2000);
        liftExtension.setTarget(150);
        wrist.setPosition(0.4);
        clawServo.setPosition(0.95);
        clawOpen = false;
    }

    public void sampleCollectionPos() {
        trig = true;
        basket = false;
        highTarget = true;
        lowTarget = false;
        liftRotation.setTarget(667);
        liftExtension.setTarget(5);
        wrist.setPosition(0.15);
        clawServo.setPosition(0.3);
    }
    public void postSpecimenScoringPos() {
        trig = false;
        basket = false;
        if(liftNotAtPosition){
        liftRotation.setTarget(1332);
        }
        if(liftRotation.getLiftMotor().getCurrentPosition() <= 1350)
        {
            clawServo.setPosition(0.3);
            liftNotAtPosition = false;
            scoreSpecimen = false;
        }

    }

    public void outputLog() {

    }
}