package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.LinearMotorController;
import org.firstinspires.ftc.teamcode.LogFile;
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
    final int MINIMUM_DEGREES = 56;
    NewMecanumDrive drive;
    InputHandler inputHandler;
    Vector3d mecanumController;
    LinearMotorController liftRotation, liftExtension;
    DcMotor leftSpoolMotor, rightSpoolMotor;
    Servo clawServo, wrist, wristRotation;
    Servo rightHangServo, leftHangServo;
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.8;     // Maximum rotational position
    static final double MIN_POS     =  0.3; // Minimum rotational position
    boolean slowMode = false;
    boolean trig = false;
    boolean experimental = false;
    boolean secondHang = false;
    boolean stopStrain = false;
    boolean ran = false;


    double driveCoefficient = 1;
    IMU imu;
    Orientation or;
    IMU.Parameters myIMUparameters;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime headingTimer = new ElapsedTime();
    ElapsedTime clawTimer;
    ElapsedTime hangTimer = new ElapsedTime();
    double deltaTime;
    double previousTime;
    boolean liftNotAtPosition = true;
    boolean scoreSpecimen = false;

    double globalIMUHeading;
    double headingError = 0;
    boolean resetIMU = false;
    boolean hanging = false;
    boolean basket = false;

    TouchSensor rotationTouchSensor;

    //TRIG BELOW:

    double theta;
    static final double HIGH_TARGET = 2.5;
    static final double LOW_TARGET = 6;
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
    public static boolean logDetails = false;
    LogFile detailsLog;

    @Override
    public void init() {
        //init dpad hashmap with each dpad value as unpressed
        imu = hardwareMap.get(IMU.class, "imu");
        clawServo = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        wristRotation = hardwareMap.get(Servo.class, "wristRotation");
        rotationTouchSensor = hardwareMap.get(TouchSensor.class, "rotationSensor");
        leftSpoolMotor = hardwareMap.get(DcMotor.class, "leftSpoolMotor");
        rightSpoolMotor = hardwareMap.get(DcMotor.class, "rightSpoolMotor");

        rightSpoolMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSpoolMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSpoolMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSpoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSpoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSpoolMotor.setPower(0);
        rightSpoolMotor.setPower(0);

        leftHangServo = hardwareMap.get(Servo.class, "leftHangServo");
        rightHangServo = hardwareMap.get(Servo.class, "rightHangServo");


        /*
        imu.initialize(new IMU.Parameters( new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));*/
        imu.resetYaw();
        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        globalIMUHeading = or.thirdAngle;

        //initialize drive, empty Vector as we are not using the Roadrunner drive methods in TeleOp
        drive = new NewMecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), 0), detailsLog, logDetails);
        //TODO: Update reset on liftExtension for competition day
        liftRotation = new LinearMotorController(hardwareMap, "swing",
                3000, false, true);


        clawTimer = new ElapsedTime();

        //Initialize gamepad handler
        inputHandler = InputAutoMapper.normal.autoMap(this);

        //values for gamepad joystick values represented as a vector3D
        mecanumController = new Vector3d();

        //Initialize encoder wheels
        //paralellEncoder = hardwareMap.get(DcMotor.class, "parallelEncoder");
        //paralellEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void init_loop(){
        telemetry.addData("Press Y to enable experimental", "");
        telemetry.addData("Experimental: ", experimental);
        if(inputHandler.up("D1:Y")){
            experimental = !experimental;
        }
        telemetry.update();
        inputHandler.loop();
    }

    public void start(){
        ElapsedTime startTimer = new ElapsedTime();
        while(startTimer.milliseconds() <= 300) {
            clawServo.setPosition(0.95);
            wrist.setPosition(1);
            wristRotation.setPosition(0.5);
            leftHangServo.setPosition(0.55);
            rightHangServo.setPosition(0.475);

        }
        liftExtension = new LinearMotorController(hardwareMap, "slide",
                1390, true, true);
        initializeArm();
    }

    @Override
    public void loop() {
        if(hanging && liftRotation.getLiftMotor().getCurrentPosition() <= 1000 && hangTimer.milliseconds() > 1500){
            if(!ran) {
                liftRotation.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftRotation.getLiftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ran = true;
            }
            if(!stopStrain) {
                if (!rotationTouchSensor.isPressed()) {
                    liftRotation.getLiftMotor().setPower(-1);
                } else {
                    liftRotation.getLiftMotor().setPower(0);
                }
            } else {
                liftRotation.getLiftMotor().setPower(0);
            }
        } else {
            if(trig) {
                liftExtension.update(liftExtControl, LIFT_EXT_COEFFICIENT, false);
                wristRotation.setPosition(wristRotation.getPosition()+gamepad2.left_stick_x*0.033);
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
                liftExtension.update(liftExtControl, LIFT_EXT_COEFFICIENT, false);
                liftRotation.update(liftRotControl, LIFT_ROT_COEFFICIENT, rotationTouchSensor.isPressed());
                wristRotation.setPosition(0.5);
            }
        }
        if(hanging && rotationTouchSensor.isPressed()){
            wrist.setPosition(0.225);
        }
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


        if(basket) {
            liftExtension.setTickLimit(1390);
        } else {
            liftExtension.setTickLimit(1310);
        }

        telemetry.addData("Rot: ", liftRotation.target);
        telemetry.addData("Ext: ", liftExtension.target);

        telemetry.addData("Rot Actual: ", liftRotation.getLiftMotor().getCurrentPosition());
        telemetry.addData("Ext Actual: ", liftExtension.getLiftMotor().getCurrentPosition());

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
        if(!secondHang) {
            mecanumController = new Vector3d((gamepad1.right_stick_x * driveCoefficient), (gamepad1.right_stick_y * driveCoefficient), (gamepad1.left_stick_x * driveCoefficient));
        } else {
                leftSpoolMotor.setPower(gamepad1.right_stick_y);
                rightSpoolMotor.setPower(gamepad1.right_stick_y);
                if(Math.abs(gamepad1.right_stick_y) > 0.15){
                    stopStrain = true;
            }
        }

        liftRotControl = gamepad2.left_stick_y;

        if(inputHandler.active("D2:DPAD_UP")){
            driverControlled = true;
            if(controlledTarget > 2) {
                controlledTarget -= 0.06;
            }
        }

        if(inputHandler.up("D2:B")){
            initializeArm();
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

        if(inputHandler.up("D1:DPAD_UP")){

        }

        if(inputHandler.up("D2:Y")){
            scoreSpecimen = true;
            liftNotAtPosition = true;
        }

        if(inputHandler.up("D2:LB")){
            sampleCollectionPos();
        }

        if(inputHandler.up("D2:LT")){
            driverControlled = false;
            highTarget = !highTarget;
            lowTarget = !lowTarget;
        }

        if(inputHandler.up("D2:START")){
            experimental = !experimental;
        }

        if(scoreSpecimen){
            if(!experimental) {
                postSpecimenScoringPos();
            } else {
                postSpecimenScoringPos();
            }
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
            if(!experimental) {
                specimenCollectionPos();
            } else {
                specimenCollectionPos();
            }
        }
        if(inputHandler.up("D2:RT")){
            if(!experimental) {
                preSpecimenScoringPos();
            } else {
                preSpecimenScoringPos();
            }
        }

        if(inputHandler.up("D1:RB")){
            globalIMUHeading = globalIMUHeading - Math.PI/2;
        }

        if(inputHandler.up("D1:LB")){
            globalIMUHeading = globalIMUHeading + Math.PI/2;
        }

        if(inputHandler.up("D2:RB")){
            highBasket();
        }
        if(inputHandler.up("D1:LT")){
            beginHang();
        }
        if(inputHandler.up("D1:A")){
            if(hanging){
                rightHangServo.setPosition(1);
                leftHangServo.setPosition(0);
                secondHang = true;
            }
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
            hangTimer.reset();
            liftRotation.setTickLimit(3333);
            liftRotation.setTarget(3333);
            liftExtension.setTarget(5);
            clawServo.setPosition(0.95);
            wrist.setPosition(0.4);

        }
    }
    public void thirdLevelHang()
    {
        clawServo.setPosition(0.95);
        wrist.setPosition(0.2);
        //hangMotor.setTarget(10);
    }
    public void specimenCollectionPos(){
        trig = false;
        basket = false;
        liftRotation.setTarget(886);
        liftExtension.setTarget(200);
        clawServo.setPosition(0.95);
        clawOpen = false;
        wrist.setPosition(0.5);
    }

    public void experimentalSpecimenCollectionPos(){
        trig = false;
        basket = false;
        liftRotation.setTarget(825);
        liftExtension.setTarget(0);
        clawServo.setPosition(0.3);
        clawOpen = true;
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

    public void experimentalPreSpecimenScoringPos(){
        trig = false;
        basket = false;
        wrist.setPosition(0.91);
        clawServo.setPosition(0.975);
        clawOpen = false;
        liftRotation.setTarget(3000);
        liftExtension.setTarget(0);
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

    public void experimentalPostSpecimenScoringPos(){
        trig = false;
        basket = false;
        if(liftNotAtPosition){
            liftExtension.setTarget(450);
        }
        if(liftExtension.getLiftMotor().getCurrentPosition() >= 445)
        {
            clawServo.setPosition(0.3);
            clawOpen = true;
            liftNotAtPosition = false;
            scoreSpecimen = false;
        }
    }

    public void outputLog() {

    }

    public void initializeArm() {
        liftExtension.setTarget(0);
        clawServo.setPosition(0.95);
        liftRotation.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRotation.getLiftMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRotation.getLiftMotor().setPower(-0.5);
        wrist.setPosition(0.95);
        ElapsedTime messUpTimer = new ElapsedTime();
        while (messUpTimer.milliseconds() < 5000 && !rotationTouchSensor.isPressed()) {
            if (rotationTouchSensor.isPressed()) {
                liftRotation.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftRotation.getLiftMotor().setPower(1.0);
                liftRotation.setTarget(5);
                liftRotation.getLiftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftRotation.getLiftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            }
        }
        liftRotation.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRotation.getLiftMotor().setPower(1.0);
        liftRotation.setTarget(5);
        liftRotation.getLiftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRotation.getLiftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initializeExtension() {
        liftExtension.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftExtension.getLiftMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftExtension.getLiftMotor().setPower(-0.1);
        int prev = 0;
        int current = 0;
        int deltaTicks;
        while(true){
            prev = current;
            current = liftExtension.getLiftMotor().getCurrentPosition();
            deltaTicks = Math.abs(current - prev);
            if(deltaTicks <= 20){
                liftExtension.getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftExtension.getLiftMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftExtension.getLiftMotor().setPower(1.0);
                liftExtension.setTarget(5);
                liftExtension.getLiftMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            }
        }
    }


}