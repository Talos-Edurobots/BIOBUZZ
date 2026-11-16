package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextRGBIndicator;

public class Fos {
    NextRGBIndicator indicator;
    double brightness = 1.0;
    public Fos() {
        indicator = new NextRGBIndicator(RobotController.expansionHub(), 3);
    }
    public void setColor(int color) {
        indicator.setColor(color);
    }
    public void setColor(NextRGBIndicator.Color color) {
        indicator.setColor(color);
    }
    public void showGateState(Gate.State state) {
        setColor(state == Gate.State.CLOSED ? NextRGBIndicator.Color.GREEN : NextRGBIndicator.Color.RED);
    }
    public void setBrightness(double brightness) {
        this.brightness = brightness;
        indicator.setBrightness(brightness);
    }
    public void changeBrightness(double change) {
        setBrightness(Math.max(0.0, Math.min(1.0, brightness + change)));
    }
    public NextRGBIndicator getIndicator() {
        return indicator;
    }
}
