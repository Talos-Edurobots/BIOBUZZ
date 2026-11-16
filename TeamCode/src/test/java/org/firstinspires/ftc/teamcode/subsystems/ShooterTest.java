package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.hardware.actuators.MotorVelocityConstants;
import dev.nextftc.units.Units;
import dev.nextftc.units.measuretypes.AngularVelocity;
import dev.nextftc.units.measuretypes.Angle;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.Mockito;

public class ShooterTest {
    private NextMotor mockMotor;
    private NextServo mockHood;
    private Shooter shooter;
    private AtomicLong clock;
    private AtomicReference<Double> voltage;
    private MotorVelocityConstants constants;

    @Before public void setUp() {
        mockMotor = mock(NextMotor.class);
        mockHood = mock(NextServo.class);
        constants = new MotorVelocityConstants();
        when(mockMotor.getVelocityConstants()).thenReturn(constants);
        measuredRpm(0);
        clock = new AtomicLong();
        voltage = new AtomicReference<>(12.0);
        shooter = new Shooter(mockMotor, mockHood, clock::get, voltage::get);
        Shooter.setInstance(shooter);
        clearInvocations(mockMotor);
    }
    private void measuredRpm(double rpm) {
        when(mockMotor.getEncoderVelocity()).thenReturn(Units.getRadiansPerSecond(rpm * 2 * Math.PI / 60));
    }
    private void tick(double seconds) {
        clock.addAndGet(Math.round(seconds * 1e9));
        shooter.update();
    }
    @Test public void benchmarkGainsAndEncoderScaleConfiguredWithoutStarting() {
        assertEquals(0.141200930894, constants.getKS(), 1e-12);
        assertEquals(0.00146941352322, constants.getKV(), 1e-14);
        assertTrue(constants.getKP() > 0);
        assertEquals(0, constants.getKI(), 0);
        assertEquals(0, constants.getKD(), 0);
        assertFalse(shooter.isRunning());
        Shooter configured = new Shooter(mockMotor, mockHood, clock::get);
        ArgumentCaptor<Angle> angle = ArgumentCaptor.forClass(Angle.class);
        verify(mockMotor).setAnglePerCount(angle.capture());
        assertEquals(2 * Math.PI / 28, angle.getValue().getBaseUnitMagnitude(), 1e-12);
        assertEquals(3000, configured.getTargetRpm(), 0);
        assertEquals(0, configured.getCommandedRpm(), 0);
        assertSame(shooter, Shooter.get());
    }
    @Test public void compensationPreservesReferenceVoltageEffortAndRecoversFromInvalidReadings() {
        measuredRpm(3000);
        shooter.start();
        assertEquals(Shooter.VELOCITY_KP, constants.getKP(), 1e-12);
        voltage.set(10.0); tick(.05);
        assertEquals(Shooter.VELOCITY_KS * 1.2, constants.getKS(), 1e-12);
        assertEquals(Shooter.VELOCITY_KV * 1.2, constants.getKV(), 1e-12);
        assertEquals(Shooter.VELOCITY_KP * 1.2, constants.getKP(), 1e-12);
        voltage.set(14.0); tick(.05);
        assertEquals(Shooter.VELOCITY_KS * 12 / 14, constants.getKS(), 1e-12);
        assertEquals(Shooter.VELOCITY_KP * 12 / 14, constants.getKP(), 1e-12);
        for (double invalid : new double[]{0.0, Double.NaN, Double.POSITIVE_INFINITY}) {
            voltage.set(invalid); clearInvocations(mockMotor); tick(.05);
            assertFalse(shooter.isVoltageValid());
            assertFalse(shooter.isAtSpeed());
            verify(mockMotor).setThrottle(0.0);
            verify(mockMotor, never()).setVelocitySetpoint(any());
        }
        voltage.set(12.0); clearInvocations(mockMotor); tick(.05);
        assertTrue(shooter.isVoltageValid());
        assertEquals(12, shooter.getSupplyVoltage(), 0);
        verify(mockMotor).setVelocitySetpoint(any());
        assertEquals(Shooter.VELOCITY_KP, constants.getKP(), 1e-12);
    }

    @Test public void targetsAreRpmBoundedAndRejectNonfiniteInput() {
        shooter.adjustTargetRpm(250); assertEquals(3250, shooter.getTargetRpm(), 0);
        shooter.adjustTargetRpm(-500); assertEquals(2750, shooter.getTargetRpm(), 0);
        shooter.setTargetRpm(99999); assertEquals(Shooter.MAX_TARGET_RPM, shooter.getTargetRpm(), 0);
        shooter.setTargetRpm(-1); assertEquals(0, shooter.getTargetRpm(), 0);
        assertThrows(IllegalArgumentException.class, () -> shooter.setTargetRpm(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> shooter.setTargetRpm(Double.POSITIVE_INFINITY));
        shooter.update(); verify(mockMotor, never()).setVelocitySetpoint(any());
    }
    @Test public void rampUsesElapsedTimeAndConvertsRpmToRadiansForTheCalibratedGains() {
        shooter.start();
        tick(.02); assertEquals(30, shooter.getCommandedRpm(), 1e-9);
        tick(.08); assertEquals(150, shooter.getCommandedRpm(), 1e-9);
        ArgumentCaptor<AngularVelocity> velocity = ArgumentCaptor.forClass(AngularVelocity.class);
        verify(mockMotor, times(2)).setVelocitySetpoint(velocity.capture());
        assertEquals(150 * 2 * Math.PI / 60, velocity.getValue().getBaseUnitMagnitude(), 1e-9);
        tick(10); assertEquals(300, shooter.getCommandedRpm(), 1e-9); // stalled loop is capped
        shooter.setTargetRpm(320); tick(.1); assertEquals(320, shooter.getCommandedRpm(), 1e-9);
        shooter.setTargetRpm(0); tick(.1); assertEquals(170, shooter.getCommandedRpm(), 1e-9);
        tick(.1); tick(.1); assertEquals(0, shooter.getCommandedRpm(), 1e-9);
        verify(mockMotor, atLeastOnce()).setThrottle(0);
    }
    @Test public void restartTracksCoastingSpeedAndRepeatedStartDoesNotResetRamp() {
        measuredRpm(1200); shooter.start();
        assertEquals(1200, shooter.getCommandedRpm(), 1e-9);
        tick(.1); assertEquals(1350, shooter.getCommandedRpm(), 1e-9);
        shooter.start(); assertEquals(1350, shooter.getCommandedRpm(), 1e-9);
        shooter.stop(); assertFalse(shooter.isRunning());
        assertEquals(0, shooter.getCommandedRpm(), 0);
        assertEquals(3000, shooter.getTargetRpm(), 0);
        verify(mockMotor).setThrottle(0);
        clearInvocations(mockMotor); tick(.1);
        verify(mockMotor, never()).setVelocitySetpoint(any());
    }
    @Test public void readinessUsesActualEncoderFeedbackAndCompletedRamp() {
        measuredRpm(3000); shooter.start();
        shooter.setTargetRpm(3250);
        measuredRpm(3250); assertFalse(shooter.isAtSpeed()); // target met by measurement, but reference still ramping
        tick(.1); tick(.1); assertTrue(shooter.isAtSpeed());
        measuredRpm(3000); assertFalse(shooter.isAtSpeed());
        assertEquals(250, shooter.getSpeedErrorRpm(), 1e-9);
        when(mockMotor.getThrottle()).thenReturn(.6);
        assertEquals(.6, shooter.getAppliedPower(), 0);
        shooter.stop(); assertFalse(shooter.isAtSpeed());
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
