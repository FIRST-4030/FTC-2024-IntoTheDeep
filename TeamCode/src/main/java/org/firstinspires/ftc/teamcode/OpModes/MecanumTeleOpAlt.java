/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.LogFile;
import org.firstinspires.ftc.teamcode.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;
import org.firstinspires.ftc.teamcode.gamepad.InputAutoMapper;
import org.firstinspires.ftc.teamcode.gamepad.InputHandler;
import org.firstinspires.ftc.teamcode.math.maths.vectors.Vector3d;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Config
@TeleOp
public class MecanumTeleOpAlt extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    public static double startX = 0;
    public static double startY = 0;
    public static double targetX = 20;
    public static double targetY = 0;
    public static double target2X = 20;
    public static double target2Y = 10;
    Vector3d mecanumController;
    NewMecanumDrive drive;
    InputHandler inputHandler;
    LogFile detailsLog;
    public static boolean logDetails = false;
    double driveCoefficient = 0.75;
    boolean resetIMU = false;
    double[] dpadPowerArray = new double[4];
    double headingError = 0;
    double powerCoefficient = 1;
    boolean precisionDrive = false;
    IMU imu;
    Orientation or;
    double globalIMUHeading;

    Pose2dWrapper startPose = new Pose2dWrapper(startX, startY, Math.toRadians(90));
    Pose2dWrapper targetPose = new Pose2dWrapper(targetX, targetY, Math.toRadians(90));
    Pose2dWrapper target2Pose = new Pose2dWrapper(target2X, target2Y, Math.toRadians(90));
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        imu = hardwareMap.get(IMU.class, "imu");
//        imu.resetYaw();
 //       globalIMUHeading = or.thirdAngle;

        inputHandler = InputAutoMapper.normal.autoMap(this);
        telemetry.addData("Status", "Initialized");
        if (logDetails) { detailsLog = new LogFile(LogFile.FileType.Details,"details", "csv"); }

        drive = new NewMecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(90)), detailsLog, logDetails);
        if (!drive.controlHub.isMacAddressValid()) {
            drive.controlHub.reportBadMacAddress(telemetry, hardwareMap);
            telemetry.update();
        }
        mecanumController = new Vector3d();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        handleInput();

        imu.resetYaw();
        or = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        globalIMUHeading = or.thirdAngle;

        headingError = or.thirdAngle - globalIMUHeading;
        if(headingError > Math.PI) headingError -= 2*Math.PI;
        else if(headingError < -Math.PI) headingError += 2*Math.PI;

        telemetry.addData("error: ", headingError);

        resetIMU = drive.update(mecanumController, dpadPowerArray, headingError, resetIMU, powerCoefficient, precisionDrive);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    void handleInput() {
        inputHandler.loop();
//        mecanumController = new Vector3d((gamepad1.right_stick_x * driveCoefficient), (gamepad1.right_stick_y * driveCoefficient), (gamepad1.left_stick_x * driveCoefficient));
        mecanumController = new Vector3d((gamepad1.left_stick_x * driveCoefficient), (gamepad1.left_stick_y * driveCoefficient), (gamepad1.right_stick_x * driveCoefficient));
        if(inputHandler.up("D1:Y")) {
            goToDeposit();
        }
        if(inputHandler.up("D1:X")) {
            goToCollection();
        }
        if(inputHandler.up("D1:A")){
            goToDeposit();
            sleep(1000);
            goToCollection();
        }
        if(inputHandler.up("D1:B"))
        {
            goToDeposit();
            sleep(1000);
            goTo2ndPoint();
            sleep(1000);
            returnToFirstPoint();
            sleep(1000);
            goToCollection();
        }

    }

    void goToDeposit()
    {
        Action action4 = drive.actionBuilder(startPose.toPose2d())
                .strafeToLinearHeading(targetPose.toPose2d().position, targetPose.toPose2d().heading)
                .build();
        Actions.runBlocking(action4);

    }
    void goToCollection()
    {
        Action action5 = drive.actionBuilder(targetPose.toPose2d())
                .strafeToLinearHeading(startPose.toPose2d().position, startPose.toPose2d().heading)
                .build();
        Actions.runBlocking(action5);
    }
    void goTo2ndPoint()
    {
        Action action6 = drive.actionBuilder(targetPose.toPose2d())
                .strafeToLinearHeading(target2Pose.toPose2d().position, target2Pose.toPose2d().heading)
                .build();
        Actions.runBlocking(action6);
    }
    void returnToFirstPoint()
    {
        Action action7 = drive.actionBuilder(target2Pose.toPose2d())
                .strafeToLinearHeading(targetPose.toPose2d().position, targetPose.toPose2d().heading)
                .build();
        Actions.runBlocking(action7);
    }
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
