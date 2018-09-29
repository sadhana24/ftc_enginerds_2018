package org.firstinspires.ftc.enginerds.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.enginerds.hardware.EnginNerdsHardware;

@TeleOp(name = "ColorTest", group = "Mecanum")
public class ColorTest extends OpMode {

    public EnginNerdsHardware robot = new EnginNerdsHardware();
    ColorSensor colorSensor;    // Hardware Device Object



    public void init (){
        /**
         * Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(this, "MODE_TELEOP");


    }

    public void loop(){
        if(gamepad2.x){
            telemetry.addData("Gamepad 2", "A Button was Pressed");
            telemetry.update();

            //an array that has three values: hue, saturation, and value
            float hsvValues[] = {0F,0F,0F};

            //a reference to the hsvValues
            final float values[] = hsvValues;

            //creates a references to the colorsensor object
            colorSensor = hardwareMap.colorSensor.get("sensor_color");

            // turn the LED on in the beginning, just so user will know that the sensor is active.
            colorSensor.enableLed(true);

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", true ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            telemetry.update();

        }

    }


}
