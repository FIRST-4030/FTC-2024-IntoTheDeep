package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.GeneralConstants.PRIMARY_BOT;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.math.maths.vectors.Vector3d;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public class NewMecanumDrive {
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        // drive model parameters
        public double inPerTick;
        public double lateralInPerTick;
        public double trackWidthTicks;

        // feedforward parameters (in tick units)
        //kV: 0.00010920853856016985, kS: 0.6733835719712635
        public double kS;
        public double kV;
        public double kA;

        // path profile parameters (in inches)
        public double maxWheelVel;
        public double minProfileAccel;
        public double maxProfileAccel;

        // turn profile parameters (in radians)
        public double maxAngVel; // shared with path
        public double maxAngAccel;

        // path controller gains
        public double axialGain;
        public double lateralGain;
        public double headingGain; // shared with turn

        public double axialVelGain;
        public double lateralVelGain;
        public double headingVelGain; // shared with turn
    }

    public static String networkName;

    public static Params PARAMS = new Params();

    ControlHub controlHub = new ControlHub();

    public MecanumKinematics kinematics = null;

    public TurnConstraints defaultTurnConstraints = null;

    public VelConstraint defaultVelConstraint = null;

    public AccelConstraint defaultAccelConstraint = null;

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    public DcMotorEx liftExtender, liftRotator;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;
    public final LazyImu teleOpImu;

    public final Localizer localizer;
    public Pose2d pose;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);


    public double joystickX;
    public double joystickY;
    public double joystickR;
    public double robotAngle;

    boolean dpadInUse = false;
    public ElapsedTime correctionTimer = new ElapsedTime();

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;
        public final IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;

        public DriveLocalizer() {
            leftFront = new OverflowEncoder(new RawEncoder(NewMecanumDrive.this.leftFront));
            leftBack = new OverflowEncoder(new RawEncoder(NewMecanumDrive.this.leftBack));
            rightBack = new OverflowEncoder(new RawEncoder(NewMecanumDrive.this.rightBack));
            rightFront = new OverflowEncoder(new RawEncoder(NewMecanumDrive.this.rightFront));

            imu = lazyImu.get();

            // TODO: reverse encoders if needed
            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        @Override
        public Twist2dDual<Time> update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new Twist2dDual<>(
                        Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                        DualNum.constant(0.0, 2)
                );
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    twist.line,
                    DualNum.cons(headingDelta, twist.angle.drop(1))
            );
        }
    }

    public NewMecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;

        networkName = controlHub.getNetworkName();

        // set PARAMS based upon the network you are connected to
        setParams();

        // initialize some classes after some PARAMS are set
        initializeOtherParameters();

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (NewMecanumDrive.networkName.equals(PRIMARY_BOT)) {
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else {
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        teleOpImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new TwoDeadWheelLocalizer(hardwareMap, lazyImu.get(), PARAMS.inPerTick);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public boolean update(Vector3d control, double[] dpadPowers, double headingError, boolean reset, double powerCoefficient, boolean slowed) {
        //checks to see if any dpad buttons are pressed
        for (double power : dpadPowers) {
            if (power != 0){
                dpadInUse = true;
                break;
            }
        }
        if(reset){
            teleOpImu.get().resetYaw();
            reset = false;
        }
        //get the current robot heading to use for field centric
        robotAngle = teleOpImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        //y values are inverted
        joystickY = -1 * Math.pow(control.y, 3);
        joystickX = Math.pow(control.x, 3);
        joystickR = control.z * 0.3;
        //if the dpad is being used, use dpad booleans to mimic joystick values, and override other inputs
        if(dpadInUse){
            joystickY = dpadPowers[0] + dpadPowers[1];
            joystickX = dpadPowers[2] + dpadPowers[3];
            joystickR = 0;
            dpadInUse = false;
        }
        //scale rotX and then scale all values by a coefficient
        joystickX *= 1.1;
        joystickX *= powerCoefficient;
        joystickY *= powerCoefficient;
        joystickR *= powerCoefficient;
        if(!slowed) {
            holdHeading(headingError);
        }

        //uses either dpad or joystick to drive motors to the proper power by normalizing values to one
        //convertToFieldCentric();
        double normalization = Math.max(Math.abs(joystickX) + Math.abs(joystickY) + Math.abs(joystickR), 1);
        leftFront.setPower((joystickY + joystickX + joystickR)/normalization);
        leftBack.setPower((joystickY - joystickX + joystickR)/normalization);
        rightFront.setPower((joystickY - joystickX - joystickR)/normalization);
        rightBack.setPower((joystickY + joystickX - joystickR)/normalization);
        return reset;
    }

    public void convertToFieldCentric(){
        joystickX = joystickX * Math.cos(-robotAngle) - joystickY * Math.sin(-robotAngle);
        joystickY = joystickX * Math.sin(-robotAngle ) + joystickY * Math.cos(-robotAngle);
    }

    public void holdHeading(double headingError){
        if(Math.abs(joystickR) > 0.05){
            correctionTimer.reset();
        }
        if(correctionTimer.milliseconds() > 300){
            joystickR = headingError * 1.2;
        }
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            Pose2d error = txWorldTarget.value().minusExp(pose);


            if ((t >= timeTrajectory.duration && error.position.norm() < 2
                    && robotVelRobot.linearVel.norm() < 0.5)
                    || t >= timeTrajectory.duration + 0.5) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return twist.velocity().value();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    private void initializeOtherParameters() {

        kinematics = new MecanumKinematics(
                PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

        defaultTurnConstraints = new TurnConstraints(
                PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);

        defaultVelConstraint = new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                new AngularVelConstraint(PARAMS.maxAngVel)));

        defaultAccelConstraint = new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);
    }

    private void setParams() {

        if (networkName.equals(PRIMARY_BOT)) {

            PARAMS.inPerTick = 0.00057336671;
            PARAMS.lateralInPerTick = 0.0005854192942301951;
            PARAMS.trackWidthTicks = 28132;

            PARAMS.kS = 0.6733835719712635;
            PARAMS.kV = 0.00010920853856016985;
            PARAMS.kA = 0.000009;

            PARAMS.maxWheelVel = 43;
            PARAMS.minProfileAccel = -30;
            PARAMS.maxProfileAccel = 40;

            PARAMS.axialGain = 8.0;
            PARAMS.lateralGain = 15.0;
            PARAMS.headingGain = 15.0;

        } else {
            PARAMS.inPerTick = 0.000566864859;
            PARAMS.lateralInPerTick = 0.00046837048125481455;
            PARAMS.trackWidthTicks = 22580.573725252547;

            PARAMS.kS = 0.8162221711868276;
            PARAMS.kV = 0.00011157757280198724;
            PARAMS.kA = 0.00005;

            PARAMS.maxWheelVel = 50;
            PARAMS.minProfileAccel = -30;
            PARAMS.maxProfileAccel = 50;

            PARAMS.axialGain = 10.0;
            PARAMS.lateralGain = 2.0;
            PARAMS.headingGain = 5.0;
        }

        PARAMS.maxAngVel = Math.PI;
        PARAMS.maxAngAccel = Math.PI;

        PARAMS.axialVelGain = 0.0;
        PARAMS.lateralVelGain = 0.0;
        PARAMS.headingVelGain = 0.0;
    }
}