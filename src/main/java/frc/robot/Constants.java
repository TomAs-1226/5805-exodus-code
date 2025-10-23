package frc.robot;

/**
 * constants file where all the constant-y constants r located
 * @author florpy
 */
public final class Constants {

    //put consts here (or dont..)
    public static final double MODULE_WIDTH = 21.75;
    public static final double MODULE_LENGTH = 27.75;

    /**
     * Swerve-related constants and per-module configuration.
     *
     * <p>The Cancoder offsets are expressed in degrees and represent the
     * physical heading of each wheel when the robot is considered to be
     * "zeroed" (pointing straight ahead).  Tune these values to match the
     * robot in the real world.</p>
     */
    public static final class Swerve {
        /** Cancoder offset for the front-left module (degrees). */
        public static final double FL_CANCODER_OFFSET_DEG = 0.0;
        /** True if the front-left Cancoder reports clockwise as positive. */
        public static final boolean FL_CANCODER_CLOCKWISE_POSITIVE = false;

        /** Cancoder offset for the front-right module (degrees). */
        public static final double FR_CANCODER_OFFSET_DEG = 0.0;
        /** True if the front-right Cancoder reports clockwise as positive. */
        public static final boolean FR_CANCODER_CLOCKWISE_POSITIVE = false;

        /** Cancoder offset for the back-left module (degrees). */
        public static final double BL_CANCODER_OFFSET_DEG = 0.0;
        /** True if the back-left Cancoder reports clockwise as positive. */
        public static final boolean BL_CANCODER_CLOCKWISE_POSITIVE = false;

        /** Cancoder offset for the back-right module (degrees). */
        public static final double BR_CANCODER_OFFSET_DEG = 0.0;
        /** True if the back-right Cancoder reports clockwise as positive. */
        public static final boolean BR_CANCODER_CLOCKWISE_POSITIVE = false;
    }

    /** elevator heights where ELEVATOR_HEIGHTS[index] returns the height for scoring index */
    public static final double[] ELEVATOR_HEIGHTS = {
        0,
        7.5,
        15.5,
        31.5,
        57.67
    };

    /**
     * dont instantiate this guh..
     */
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
