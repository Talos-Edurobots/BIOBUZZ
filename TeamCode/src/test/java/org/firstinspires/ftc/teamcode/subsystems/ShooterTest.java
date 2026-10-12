package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import java.util.List;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;

public class ShooterTest {

    private NextMotor mockMotor;
    private NextServo mockHood;
    private Shooter shooter;

    @Before
    public void setUp() {
        mockMotor = Mockito.mock(NextMotor.class);
        mockHood = Mockito.mock(NextServo.class);
        shooter = new Shooter(mockMotor, mockHood);
        Shooter.setInstance(shooter);
    }

    @Test
    public void singletonInstance_returnsAndOverridesSingleton() {
        assertSame(shooter, Shooter.get());
    }

    @Test
    public void initialState_defaultValuesAreCorrect() {
        assertEquals(0.0, shooter.getTargetPower(), 1e-6);
        assertEquals(0.0, shooter.getAppliedPower(), 1e-6);
        assertFalse(shooter.isRunning());
    }

    @Test
    public void setTargetPower_validValues_setsTargetPower() {
        shooter.setTargetPower(0.75);
        assertEquals(0.75, shooter.getTargetPower(), 1e-6);
    }

    @Test
    public void setTargetPower_aboveOne_clampsToOne() {
        shooter.setTargetPower(1.5);
        assertEquals(1.0, shooter.getTargetPower(), 1e-6);
    }

    @Test
    public void setTargetPower_belowZero_clampsToZero() {
        shooter.setTargetPower(-0.5);
        assertEquals(0.0, shooter.getTargetPower(), 1e-6);
    }

    @Test
    public void adjustTargetPower_positiveAndNegative_updatesAndClamps() {
        shooter.adjustTargetPower(0.5);
        assertEquals(0.5, shooter.getTargetPower(), 1e-6);

        shooter.adjustTargetPower(0.7); // 0.5 + 0.7 = 1.2 -> clamped to 1.0
        assertEquals(1.0, shooter.getTargetPower(), 1e-6);

        shooter.adjustTargetPower(-0.4); // 1.0 - 0.4 = 0.6
        assertEquals(0.6, shooter.getTargetPower(), 1e-6);

        shooter.adjustTargetPower(-1.0); // 0.6 - 1.0 = -0.4 -> clamped to 0.0
        assertEquals(0.0, shooter.getTargetPower(), 1e-6);
    }

    @Test
    public void start_setsRunningAndResetsMotorThrottle() {
        shooter.start();

        assertTrue(shooter.isRunning());
        assertEquals(0.0, shooter.getAppliedPower(), 1e-6);
        verify(mockMotor).setThrottle(0.0);
    }

    @Test
    public void update_whenNotRunning_doesNotChangePowerOrCallMotor() {
        shooter.setTargetPower(0.8);
        shooter.update();

        assertEquals(0.0, shooter.getAppliedPower(), 1e-6);
        verify(mockMotor, never()).setThrottle(anyDouble());
    }

    @Test
    public void update_whenRunning_rampsPowerUpTowardsTarget() throws InterruptedException {
        shooter.setTargetPower(1.0);
        shooter.start(); // initial motor throttle set to 0.0

        // Small pause to ensure non-zero elapsed time between System.nanoTime() calls
        Thread.sleep(50);
        shooter.update();

        double currentPower = shooter.getAppliedPower();
        assertTrue("Power should be greater than 0 after ramp update", currentPower > 0.0);
        assertTrue("Power should not exceed target", currentPower <= 1.0);

        ArgumentCaptor<Double> captor = ArgumentCaptor.forClass(Double.class);
        verify(mockMotor, atLeastOnce()).setThrottle(captor.capture());
        assertEquals(currentPower, captor.getValue(), 1e-6);
    }

    @Test
    public void update_whenTargetDecreased_rampsPowerDown() throws InterruptedException {
        shooter.setTargetPower(1.0);
        shooter.start();

        // First update to ramp power up
        Thread.sleep(50);
        shooter.update();
        double initialPower = shooter.getAppliedPower();

        // Reduce target power below current power
        shooter.setTargetPower(0.01);
        Thread.sleep(50);
        shooter.update();

        double updatedPower = shooter.getAppliedPower();
        assertTrue("Power should ramp down when target decreases", updatedPower < initialPower);
    }

    @Test
    public void stop_resetsRunningAndAppliedPower_preservesTargetPower() {
        shooter.setTargetPower(0.8);
        shooter.start();

        shooter.stop();

        assertFalse(shooter.isRunning());
        assertEquals(0.0, shooter.getAppliedPower(), 1e-6);
        assertEquals(0.8, shooter.getTargetPower(), 1e-6);
        verify(mockMotor, atLeastOnce()).setThrottle(0.0);
    }

    @Test
    public void changeHoodPosition_withinBounds_enablesAndSetsPosition() {
        shooter.changeHoodPosition(0.2); // starts at 0.5 -> 0.7

        verify(mockHood).enable();
        ArgumentCaptor<Double> captor = ArgumentCaptor.forClass(Double.class);
        verify(mockHood).setPosition(captor.capture());
        assertEquals(0.7, captor.getValue(), 1e-6);
    }

    @Test
    public void changeHoodPosition_cumulativeUpdates_accumulatesCorrectly() {
        shooter.changeHoodPosition(0.2); // 0.5 -> 0.7
        shooter.changeHoodPosition(0.1); // 0.7 -> 0.8

        ArgumentCaptor<Double> captor = ArgumentCaptor.forClass(Double.class);
        verify(mockHood, times(2)).setPosition(captor.capture());
        List<Double> positions = captor.getAllValues();
        assertEquals(0.7, positions.get(0), 1e-6);
        assertEquals(0.8, positions.get(1), 1e-6);
    }

    @Test
    public void changeHoodPosition_upperBound_clampsToOne() {
        shooter.changeHoodPosition(0.8); // 0.5 + 0.8 = 1.3 -> clamped to 1.0

        verify(mockHood).enable();
        ArgumentCaptor<Double> captor = ArgumentCaptor.forClass(Double.class);
        verify(mockHood).setPosition(captor.capture());
        assertEquals(1.0, captor.getValue(), 1e-6);
    }

    @Test
    public void changeHoodPosition_lowerBound_clampsToHalf() {
        shooter.changeHoodPosition(-0.3); // 0.5 - 0.3 = 0.2 -> clamped to 0.5

        verify(mockHood).enable();
        ArgumentCaptor<Double> captor = ArgumentCaptor.forClass(Double.class);
        verify(mockHood).setPosition(captor.capture());
        assertEquals(0.5, captor.getValue(), 1e-6);
    }
}
