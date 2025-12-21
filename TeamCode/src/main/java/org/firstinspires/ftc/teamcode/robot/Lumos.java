package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Lumos - Control class for goBILDA RGB Indicator Light (3118-0808-0002)
 *
 * The RGB Indicator Light responds to standard Servo PWM signals.
 * As the signal changes, the displayed color smoothly transitions through a gradient.
 * - Signal below 1100µsec (FTC: ~0.277): Light turns off
 * - Signal above 1900µsec (FTC: ~0.722): Light turns solid white
 *
 * Color Reference:
 * Off: 0.0
 * Red: 0.277
 * Orange: 0.333
 * Yellow: 0.388
 * Sage: 0.444
 * Green: 0.500
 * Azure: 0.555
 * Blue: 0.611
 * Indigo: 0.666
 * Violet: 0.722
 * White: 1.0
 */
// TODO: for some reason this code isn't working, but the light works with the Servo programmer.
public class Lumos {

    // RGB Indicator Light
    public Servo light;

    // Color Constants (FTC values 0-1)
    public static final double OFF = 0.0;
    public static final double RED = 0.277;
    public static final double ORANGE = 0.333;
    public static final double YELLOW = 0.388;
    public static final double SAGE = 0.444;
    public static final double GREEN = 0.500;
    public static final double AZURE = 0.555;
    public static final double BLUE = 0.611;
    public static final double INDIGO = 0.666;
    public static final double VIOLET = 0.722;
    public static final double WHITE = 1.0;

    /**
     * Initialize the Lumos light controller
     * @param hardwareMap The hardware map from the OpMode
     */
    public Lumos(HardwareMap hardwareMap) {
        light = hardwareMap.get(Servo.class, "light");
    }

    /**
     * Set the light to a specific color using a position value (0.0 - 1.0)
     * @param position The servo position (0.0 = off, 1.0 = white)
     */
    public void setColor(double position) {
        light.setPosition(position);
    }

    /**
     * Turn the light off
     */
    public void off() {
        light.setPosition(OFF);
    }

    /**
     * Set the light to red
     */
    public void red() {
        light.setPosition(RED);
    }

    /**
     * Set the light to orange
     */
    public void orange() {
        light.setPosition(ORANGE);
    }

    /**
     * Set the light to yellow
     */
    public void yellow() {
        light.setPosition(YELLOW);
    }

    /**
     * Set the light to sage (yellow-green)
     */
    public void sage() {
        light.setPosition(SAGE);
    }

    /**
     * Set the light to green
     */
    public void green() {
        light.setPosition(GREEN);
    }

    /**
     * Set the light to azure (cyan)
     */
    public void azure() {
        light.setPosition(AZURE);
    }

    /**
     * Set the light to blue
     */
    public void blue() {
        light.setPosition(BLUE);
    }

    /**
     * Set the light to indigo
     */
    public void indigo() {
        light.setPosition(INDIGO);
    }

    /**
     * Set the light to violet (purple)
     */
    public void violet() {
        light.setPosition(VIOLET);
    }

    /**
     * Set the light to white
     */
    public void white() {
        light.setPosition(WHITE);
    }

    /**
     * Get the current color position
     * @return The current servo position (0.0 - 1.0)
     */
    public double getPosition() {
        return light.getPosition();
    }
}

