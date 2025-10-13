package frc.robot;

/**
 * constants file where all the constant-y constants r located
 * @author florpy
 */
public final class Constants {

    //put consts here (or dont..)
    public static final double MODULE_WIDTH = 21.75;
    public static final double MODULE_LENGTH = 27.75;

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
