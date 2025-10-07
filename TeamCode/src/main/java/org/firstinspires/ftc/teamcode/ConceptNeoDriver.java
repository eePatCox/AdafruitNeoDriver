package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name="Concept: NeoDriver", group = "Concept")
//@Disabled
public class ConceptNeoDriver extends LinearOpMode {

    private NeoDriver LED = null;

    @Override
    public void runOpMode() {
        LED = hardwareMap.get(NeoDriver.class, "led");
        LED.initializeNeoPixels();


        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            for (int pix = 0; pix < 16; pix++) {
                LED.setPixelColor(pix, 0, 127, 0);
            }
            LED.show();

            telemetry.update();
        }
    }
}
