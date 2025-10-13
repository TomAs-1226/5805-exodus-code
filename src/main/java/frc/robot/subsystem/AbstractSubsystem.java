package frc.robot.subsystem;

/**
 * shrimple abstract subsystem
 * @author florpy
 */
public abstract class AbstractSubsystem {

    /**
     * construct new subsystem
     */
    public AbstractSubsystem() {}

    /**
     * this runs when the subsystem is to be started
     * @apiNote called by {@link SubsystemManager} automatically
     */
    public abstract void start();

    /**
     * steps this subsystem; should be automatically called by {@link SubsystemManager}
     * @see SubsystemManager#update()
     */
    public abstract void update();

    /**
     * this runs when the subsystem is to be stopped
     * @apiNote called by {@link SubsystemManager} automatically
     */
    public abstract void stop();
    
}
