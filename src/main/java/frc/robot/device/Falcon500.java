package frc.robot.device;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Simple TalonFX wrapper for a Falcon 500 motor.
 */
public class Falcon500 extends TalonFX {
    /**
     * Creates a new Falcon 500 on the specified device id.
     *
     * @apiNote the default canbus is "rio", see {@link #Falcon500(int, String)}
     */
    public Falcon500(int id) {
        super(id);
    }

    /**
     * Constructs a new Falcon 500 on the specified device id and CAN bus.
     *
     * @see #Falcon500(int)
     */
    public Falcon500(int id, String canBus) {
        super(id, canBus);
    }

    /**
     * Constructs a new Falcon 500 on the specified device id and CAN bus.
     */
    public Falcon500(int id, CANBus canBus) {
        super(id, canBus);
    }
}
