package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoInteractions;

import dev.nextftc.hardware.actuators.NextCRServo;
import dev.nextftc.hardware.actuators.NextServo;
import org.junit.Test;

public class ServoSubsystemTest {
    @Test
    public void gateKeepsToggleStateSeparateFromDebugPositionAndPwm() {
        NextServo servo = mock(NextServo.class);
        Gate gate = new Gate(servo);

        assertEquals(Gate.State.CLOSED, gate.getState());
        verifyNoInteractions(servo);

        gate.activate();
        assertEquals(Gate.State.OPEN, gate.getState());
        verify(servo).setPosition(0.6);

        gate.disablePwm();
        gate.setPosition(0.8);
        assertEquals(Gate.State.OPEN, gate.getState());
        verify(servo).disable();
        verify(servo, never()).setPosition(0.8);

        gate.enablePwm();
        verify(servo).enable();
        verify(servo).setPosition(0.8);

        gate.changePosition(0.5);
        verify(servo).setPosition(1.0);
        gate.toggle();
        assertEquals(Gate.State.CLOSED, gate.getState());
        verify(servo).setPosition(0.4);
    }

    @Test
    public void flowerMovesBetweenItsTwoPositions() {
        NextServo servo = mock(NextServo.class);
        Flower flower = new Flower(servo);

        assertEquals(Flower.State.INACTIVE, flower.getState());
        verifyNoInteractions(servo);

        flower.activate();
        assertEquals(Flower.State.ACTIVE, flower.getState());
        verify(servo).setPosition(0.6);

        flower.toggle();
        assertEquals(Flower.State.INACTIVE, flower.getState());
        verify(servo).setPosition(0.4);

        flower.changePosition(-1.0);
        assertEquals(Flower.State.INACTIVE, flower.getState());
        verify(servo).setPosition(0.0);
    }

    @Test
    public void wheelClampsPowerAndStops() {
        NextCRServo servo = mock(NextCRServo.class);
        CRWheel wheel = new CRWheel(servo);
        verifyNoInteractions(servo);

        wheel.setPower(2.0);
        wheel.setPower(-2.0);
        wheel.stop();

        verify(servo).setPower(1.0);
        verify(servo).setPower(-1.0);
        verify(servo).setPower(0.0);
        assertThrows(IllegalArgumentException.class, () -> wheel.setPower(Double.NaN));
    }
}
