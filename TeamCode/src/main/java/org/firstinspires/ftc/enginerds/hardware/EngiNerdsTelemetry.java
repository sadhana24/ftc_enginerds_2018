package org.firstinspires.ftc.enginerds.hardware;

//------------------------------------------------------------------------------
// EngiNerdsTelemetry
//------------------------------------------------------------------------------

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Provide telemetry provided by the EngiNerds Hardware class.
 *
 * Insert this class between a custom op-mode and the PushBotHardware class to
 * display telemetry available from the hardware class.
 *
 *
 * Telemetry Keys
 *     00 - Important message; sometimes used for error messages.
 *     01 - The power being sent to the left drive's motor controller and the
 *          encoder count returned from the motor controller.
 *     02 - The power being sent to the right drive's motor controller and the
 *          encoder count returned from the motor controller.
 *     03 - The power being sent to the left arm's motor controller.
 *     04 - The position being sent to the left and right hand's servo
 *          controller.
 *     05 - The negative value of gamepad 1's left stick's y (vertical; up/down)
 *          value.
 *     06 - The negative value of gamepad 1's right stick's y (vertical;
 *          up/down) value.
 *     07 - The negative value of gamepad 2's left stick's y (vertical; up/down)
 *          value.
 *     08 - The value of gamepad 2's X button (true/false).
 *     09 - The value of gamepad 2's Y button (true/false).
 *     10 - The value of gamepad 1's left trigger value.
 *     11 - The value of gamepad 1's right trigger value.
 */
public class EngiNerdsTelemetry {

   public OpMode opMode;
   public EnginNerdsHardware eHardware;

    /**
     * Construct the class.
     * The system calls this member when the class is instantiated.
     */
    public EngiNerdsTelemetry(OpMode opMode, EnginNerdsHardware eHardware) {
        this.opMode = opMode;
        this.eHardware = eHardware;
    }

    /**
     * Update the telemetry with current values from the base class.
     */
    public void updateTelemetry() {
        if ( (eHardware.isWarningGenerated()))
        {
            setFirstMessage(eHardware.GetWarningMessage());
        }
        //
        // Send telemetry data to the driver station.
        //
        getTelemetry().addData
            ( "01"
            , "Left Drive: "
                + eHardware.getLeftDrivePower()
                + ", "
                + eHardware.getLeftEncoderCount()
            );
        getTelemetry().addData
            ( "02"
            , "Right Drive: "
                + eHardware.getRightDrivePower()
                + ", "
                + eHardware.getRightEncoderCount()
            );


    } // updateTelemetry

    /**
     * Update the telemetry with current gamepad readings.
     */
    public void updateGamepadTelemetry() {
        //
        // Send telemetry data concerning gamepads to the driver station.
        //
        getTelemetry().addData ("05", "GP1 Left: " + -getGamepad1().left_stick_y);
        getTelemetry().addData ("06", "GP1 Right: " + -getGamepad1().right_stick_y);
        getTelemetry().addData ("07", "GP2 Left: " + -getGamepad2().left_stick_y);
        getTelemetry().addData ("08", "GP2 X: " + getGamepad2().x);
        getTelemetry().addData ("09", "GP2 Y: " + getGamepad2().y);
        getTelemetry().addData ("10", "GP1 LT: " + getGamepad1().left_trigger);
        getTelemetry().addData ("11", "GP1 RT: " + getGamepad1().right_trigger);
        getTelemetry().addData ("12", "GP2 LT: " + getGamepad2().left_trigger);
        getTelemetry().addData ("13", "GP2 RT: " + getGamepad2().right_trigger);
        getTelemetry().addData ("14", "GP2 LB: " + getGamepad2().left_bumper);
        getTelemetry().addData ("15", "GP2 RB: " + getGamepad2().right_bumper);
    }

    /**
     * Update the telemetry's first message with the specified message.
     */
    public void setFirstMessage(String p_message) {
        getTelemetry().addData ( "00", p_message);
    }

    /**
     * Update the telemetry's first message to indicate an error.
     */
    public void setErrorMessage(String p_message){
        setFirstMessage("ERROR: " + p_message);
    }

    /**
     * @return
     */
    public Telemetry getTelemetry() {
        return getTelemetry();
    }

    /**
     * @return
     */
    public Gamepad getGamepad1() {
        return opMode.gamepad1;
    }

    /**
     * @return
     */
    public Gamepad getGamepad2() {
        return opMode.gamepad2;
    }

} // EngiNerdsTelemetry
