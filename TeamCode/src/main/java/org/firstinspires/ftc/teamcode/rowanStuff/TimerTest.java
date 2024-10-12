package org.firstinspires.ftc.teamcode.rowanStuff;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TimerTest (Blocks to Java)")
public class TimerTest extends LinearOpMode {

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        ElapsedTime Timer1;

        // Put initialization blocks here.
        Timer1 = new ElapsedTime();
        Timer1.reset();
        //waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.addData("Timer1",Timer1.seconds());
                telemetry.update();
            }
        }
    }
}
