package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TickTest extends OpMode {

    Encoder perp;
    Encoder par;
    @Override
    public void init() {
        perp = hardwareMap.get(Encoder.class, "perp");
        par = hardwareMap.get(Encoder.class, "par");
    }

    @Override
    public void loop() {
        telemetry.addData("Perp ticks: ", perp.getPositionAndVelocity().position);
        telemetry.addData("Par ticks: ", par.getPositionAndVelocity().position);
    }
}
