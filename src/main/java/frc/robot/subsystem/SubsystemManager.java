package frc.robot.subsystem;

import java.util.ArrayList;
import java.util.function.Supplier;

/**
 * manages subsystems 
 * im certain wpilib has smthn 4 this.. but then i wouldnt learn anything
 * @author florpy
 */
public final class SubsystemManager {

    /**
     * all active subsystems
     */
    private static final ArrayList<AbstractSubsystem> ACTIVE_SUBSYSTEMS = new ArrayList<>();
    
    /**
     * do not
     */
    private SubsystemManager() { 
        throw new UnsupportedOperationException("nuh uh"); 
    };

    /**
     * registers a subsystem
     * @param <T> subsystem type T
     * @param constructor constructor (use method refs)
     * @return newly constructed & registered subsystem of subsystem type T
     */
    public static <T extends AbstractSubsystem> T registerSubsystem(Supplier<T> constructor) {
        T subsystem = constructor.get();
        ACTIVE_SUBSYSTEMS.add(subsystem);
        return subsystem;
    }

    /**
     * removes subsystem from registry, stopping if it succeeded
     * @param <T> subsystem type T
     * @param subsystem subsystem
     * @apiNote dont ever call this unless u kno wht ur doing.. made 4 completionary purposes
     */
    public static <T extends AbstractSubsystem> void remove(T subsystem) {
        boolean result = ACTIVE_SUBSYSTEMS.remove(subsystem);
        if (result) {
            subsystem.stop();
        }
    }

    /**
     * runs all startups
     */
    public static void start() {
        for (AbstractSubsystem subsystem : ACTIVE_SUBSYSTEMS) {
            subsystem.start();
        }
    }

    /**
     * runs all stops
     */
    public static void stop() {
        for (AbstractSubsystem subsystem : ACTIVE_SUBSYSTEMS) { 
            subsystem.stop();
        }
    }

    /**
     * updates all subsystems (call periodically)
     */
    public static void update() {
        for (AbstractSubsystem subsystem : ACTIVE_SUBSYSTEMS) {
            subsystem.update();
        }
    }

}
