package frc.robot.device;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.RobotMath;

/**
 * useless wrapper class that doesnt do much 
 * and likely already has some existing wpilib version
 * @author florpy
 */
public class KrakenX60 extends TalonFX {

    /** absolute max state */
    private double maxState = 1.0;
    
    /**
     * creates a new krakenx60 on the specified device id
     * @param id device id
     * @apiNote the default canbus is "rio", see {@link #KrakenX60(int, String)}
     */
    public KrakenX60(int id) {
        super(id);
    }

    /**
     * constructs a new krakenx60 on the specified deviced id & canbus <br>
     * (this rlly shouldnt need 2 be used over {@link #KrakenX60(int)})
     * @param id device id
     * @param canBus can bus name
     * @see #KrakenX60(int)
     * @see TalonFX#TalonFX(int, String) TalonFX(int, String) for default can bus names
     */
    public KrakenX60(int id, String canBus) {
        super(id, canBus);
    }

    /**
     * constructs a new krakenx60 on specified device id & canbus
     * @param id device id
     * @param canBus can bus
     */
    public KrakenX60(int id, CANBus canBus) {
        super(id, canBus);
    }

    /**
     * returns max motor state (absolute)
     * @return max motor state
     */
    public double getMaxState() {
        return this.maxState;
    }

    /**
     * sets max motor state [0.0, 1.0]
     * @param max max state [0.0, 1.0]
     * @apiNote clamps max state value between 0 & 1 for safety
     */
    public void setMaxState(double max) {
        max = RobotMath.clamp(max, 0.0, 1.0);
    }

    /**
     * sets the motor speed
     * @param speed desired speed (rlly state) from [-max, max]
     * @apiNote speed gets absolute clamped by the {@link #maxState max state}
     */
    @Override
    public void set(double speed) {
        super.set(RobotMath.clamp(speed, -maxState, maxState));
    }

}
