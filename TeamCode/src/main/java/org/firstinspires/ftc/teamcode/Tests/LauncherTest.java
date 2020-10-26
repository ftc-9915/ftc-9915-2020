package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Launcher Test", group="test")
public class LauncherTest extends OpMode {

    DcMotor launcherMotor;
    double power;
    // Amount of adjustment to be made to the motor power
    double increment;
    // Ensures that the adjustments are made each time the gamepad buttons are pressed rather than each time through loop
    boolean buttonReleased;

    @Override
    public void init() {
        launcherMotor = hardwareMap.dcMotor.get("launcherMotor");
        // Make the motor's speed constant using encoders
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialization values
        power = 0.0;
        increment = 0.01;
        buttonReleased = true;
    }

    @Override
    public void loop() {
        // Controls and Information
        telemetry.addData("Motor Power", power);
        telemetry.addData("Power Adjustment Increment", increment);
        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addLine(" - Motor Power -");
        telemetry.addData("Set Motor Power to Full Power (1.0)", "button A");
        telemetry.addData("Stop Motor (power 0.0)", "button B");
        telemetry.addData("Add Increment to Motor Power", "D-pad up");
        telemetry.addData("Subtract Increment to Motor Power", "D-pad down");
        telemetry.addLine("- Power Increment -");
        telemetry.addData("Increase Increment by 0.001", "Right Bumper");
        telemetry.addData("Decrease Increment by 0.001", "Left Bumper");

        // Run at full power or stop
        if(gamepad1.a) {
            power = 1.0;
        }
        if(gamepad1.b) {
            power = 0.0;
        }

        // Adjusting motor power based on increment value
        if(gamepad1.dpad_up && buttonReleased) {
            power = Range.clip(power + increment, 0.0, 1.0);
            buttonReleased = false;
        }
        if(gamepad1.dpad_down && buttonReleased) {
            power = Range.clip(power - increment, 0.0, 1.0);
            buttonReleased = false;
        }

        // Adjusting increment value
        // (this does not affect the motor power until the increment is applied to the motor power)
        if(gamepad1.right_bumper && buttonReleased) {
            increment += 0.01;
            buttonReleased = false;
        }
        if(gamepad1.left_bumper && buttonReleased) {
            increment = Math.max(increment - 0.01, 0.0);
            buttonReleased = false;
        }

        // Do not adjust values again until after buttons are released (and pressed again) so the
        // adjustments are made each time the gamepad buttons are pressed rather than each time through loop
        if(!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.left_bumper && !gamepad1.right_bumper) {
            buttonReleased = true;
        }

        launcherMotor.setPower(power);

    }
}
