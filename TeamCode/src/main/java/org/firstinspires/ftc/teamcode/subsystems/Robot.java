package org.firstinspires.ftc.teamcode.subsystems;

/** Per-OpMode state. A failed construction cleans up every initialized subsystem. */
public final class Robot implements AutoCloseable {
    public Drivetrain drivetrain;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Gate gate;
    public Flower flower;
    public CRWheel crWheel;
    public Fos fos;
    public Robot() {
        try {
            drivetrain=new Drivetrain(); intake=new Intake(); shooter=new Shooter(); fos=new Fos();
            turret=new Turret(); gate=new Gate(); flower=new Flower(); crWheel=new CRWheel();
        } catch (RuntimeException failure) {
            try { close(); } catch (RuntimeException cleanup) { failure.addSuppressed(cleanup); }
            throw failure;
        }
    }
    @Override public void close() {
        RuntimeException failure=null;
        Runnable[] actions={
            ()->{if(drivetrain!=null)drivetrain.stop();}, ()->{if(intake!=null)intake.stop();},
            ()->{if(shooter!=null)shooter.stop();}, ()->{if(crWheel!=null)crWheel.stop();},
            ()->{if(turret!=null)turret.deactivate();}
        };
        for(Runnable action:actions) try { action.run(); }
        catch(RuntimeException error) { if(failure==null)failure=error;else failure.addSuppressed(error); }
        if(failure!=null)throw failure;
    }
}
