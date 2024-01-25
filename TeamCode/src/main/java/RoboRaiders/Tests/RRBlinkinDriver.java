package RoboRaiders.Tests;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

@ServoType(flavor = ServoFlavor.CUSTOM, usPulseLower = 500, usPulseUpper = 2500)
@DeviceProperties(xmlTag = "RevBlinkinLedDriver", name = "@string/rev_blinkin_name", description = "@string/rev_blinkin_description",
        builtIn = true, compatibleControlSystems = ControlSystem.REV_HUB)

public class RRBlinkinDriver extends RevBlinkinLedDriver {
    protected ServoControllerEx controller;
    private final int port;
    protected final static double PULSE_WIDTH_INCREMENTOR = 0.0005;
    protected final static double BASE_SERVO_POSITION = 505 * PULSE_WIDTH_INCREMENTOR;
    protected final static int PATTERN_OFFSET = 10;

    /**
     * RRBlinkinLedDriver
     *
     * @param controller A REV servo controller
     * @param port the port that the driver is connected to
     */
    public RRBlinkinDriver(ServoControllerEx controller, int port)
    {
        super(controller, port);
        this.controller = controller;
        this.port = port;
    }

    public ServoControllerEx getController() { return this.controller; }

    public int getPort() { return this.port; }

    public double getControllerPosition() { return controller.getServoPosition(this.port); }

    public int getOrdinal(double pwm) {
        int ordinal = 0;
        ordinal = (int) (((pwm-BASE_SERVO_POSITION) / PULSE_WIDTH_INCREMENTOR) / PATTERN_OFFSET);
        if (ordinal < 0) ordinal = 0;
        else if (ordinal > 100) ordinal = 100;

        return ordinal;
    }
}
