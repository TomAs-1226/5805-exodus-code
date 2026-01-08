package frc.robot;

/**
 * constants file where all the constant-y constants r located
 * @author florpy
 */
public final class Constants {

    /** elevator heights where ELEVATOR_HEIGHTS[index] returns the height for scoring index */
    public static final double[] ELEVATOR_HEIGHTS = {
        0,
        6.5,
        14.5,
        30.5,
        57.67
    };

    /** Extra height (in) for L4 when in algae mode */
    public static final double ALGAE_L4_OFFSET_IN = 3.0;

    // ===== Elevator motion constraints (rot/s and rot/s^2) =====
    // Base profile (used for everything except Algae L4 boost)
    public static final double ELEVATOR_BASE_MAX_VEL_ROT_PER_S  = 90.0 ;
    public static final double ELEVATOR_BASE_MAX_ACC_ROT_PER_S2 = 180.0;

    // Faster profile ONLY when in Algae mode going to L4 (+offset) – keeps velocity high
    public static final double ALGAE_L4_MAX_VEL_ROT_PER_S       = 115.0;
    public static final double ALGAE_L4_MAX_ACC_ROT_PER_S2      = 250.0;
    // Slower profile when moving DOWN so it doesn't slam the floor
    public static final double ELEVATOR_DOWN_MAX_VEL_ROT_PER_S  = 70.0;
    public static final double ELEVATOR_DOWN_MAX_ACC_ROT_PER_S2 = 140.0;
    // How early to start slowing near the bottom when coming down (inches before target)
    public static final double ELEVATOR_DOWN_SLOW_WINDOW_IN     = 8.0;


    // ===== Algae L4 pre-fire logic (shoot WHILE rising, near the very top) =====
    /** Start shooting when we’re within this many inches of the L4+offset target */
    public static final double ALGAE_L4_PREFIRE_WINDOW_IN       = 2;

    /** How long to run the end-effector at full power to eject algae (seconds) */
    public static final double ALGAE_L4_SHOOT_TIME_S            = 0.7;

    private Constants() {
        throw new UnsupportedOperationException("attempted to instantiate constants class!!");
    }

    /**
     * a device (maybe should be more concrete)
     * @author florpy
     */
    public enum Device {
        ELEVATOR_LEFT(5),
        ELEVATOR_RIGHT(6),
        ELEVATOR_CORAL_INTAKE(12),
        ELEVATOR_END_EFFECTOR(13),
        ELEVATOR_END_EFFECTOR_PROX_OUT(43),
        ELEVATOR_END_EFFECTOR_PROX_IN(41),
        ELEVATOR_INTAKE_PROX(42),
        FL_DRIVE(20),
        FL_SWERVE(23),
        FL_ENCODER(3),
        FR_DRIVE(27),
        FR_SWERVE(24),
        FR_ENCODER(1),
        BL_DRIVE(26),
        BL_SWERVE(25),
        BL_ENCODER(0),
        BR_DRIVE(22),
        BR_SWERVE(21),
        BR_ENCODER(4),
        PIGEON_2(28);

        public final int ID;
        Device(int deviceId) {
            this.ID = deviceId;
        }
    }
}
