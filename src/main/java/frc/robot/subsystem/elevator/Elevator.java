package frc.robot.subsystem.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants;
import frc.robot.device.KrakenX60;
import frc.robot.subsystem.AbstractSubsystem;
import frc.robot.util.AsyncComputeTask;
import frc.robot.util.RobotMath;

// >>> imports already present <<<
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // DS keys
import edu.wpi.first.math.MathUtil;                           // clamp()
import edu.wpi.first.wpilibj.Timer;                           // <<< NEW

/**
 * blah blah blah elevator it does elevator things (it elevates, duh?)
 * this class is very ugly.. prbly dont look at it (4 ur sake)
 * @author florpy
 */
public final class Elevator extends AbstractSubsystem {
    
    /** circumference */
    private static final double CIRCUMFERENCE = 7.08661;

    /** the motor or wtvr */
    private final TalonFX MOTOR_LEFT;
    /** the other motor */
    private final TalonFX MOTOR_RIGHT;
    /** coral intake motor */
    private final KrakenX60 CORAL_INTAKE;
    /** coral end effector thingy motor */
    private final KrakenX60 END_EFFECTOR;
    /** if end effector should output */
    private boolean shouldShoot;
    /** intake algae */
    private boolean intakeAlgae;
    /** collection of proximity sensors, in order from intake -> end effector */
    private final CANrange[] PROXIMITY_SENSORS;
    /** control thingy */
    private final PositionVoltage CONTROL;
    /** feed forward */
    private final ElevatorFeedforward FEED_CTRL;
    /** profile */
    private final TrapezoidProfile PROFILE;
    /** current profile state */
    private State currentState;
    /** target profile state */
    private State targetState; 
    /** incase you need to be higher up */
    private double heightOffset;

    // <<< NEW: timer to bound algae auto-fire duration >>>
    private final Timer algaeFireTimer = new Timer();

    public boolean atHeightInches(double inches, double tolInches) {
        return Math.abs(getHeight() - inches) <= Math.abs(tolInches);
    }

    // === existing L1-only tunable ===
    private static final double DEFAULT_SHOOT_POWER_L1 = 0.75; // DS tunable

    // === NEW: global shot-power scale (used by autos like L2) ===
    private static final String  SHOT_SCALE_KEY          = "Elevator/ShotPowerScale";
    private static final double  DEFAULT_SHOT_POWER_SCALE = 1.0; // teleop/most modes

    private double getShotPowerScale() {
        // clamp for safety (0..1)
        double raw = SmartDashboard.getNumber(SHOT_SCALE_KEY, DEFAULT_SHOT_POWER_SCALE);
        return MathUtil.clamp(raw, 0.0, 1.0);
    }

    /**
     * construct elevator
     */
    public Elevator() {
        super();
        this.shouldShoot = false;
        this.intakeAlgae = false;
        this.MOTOR_LEFT = new TalonFX(Constants.Device.ELEVATOR_LEFT.ID);
        this.MOTOR_RIGHT = new TalonFX(Constants.Device.ELEVATOR_RIGHT.ID);
        TalonFXConfiguration conf = new TalonFXConfiguration();
        conf.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        conf.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        //pid might b off... idrc
        conf.Slot0 = new Slot0Configs().withGravityType(GravityTypeValue.Elevator_Static)
            .withKS(0.4)
            .withKG(0.16)
            .withKV(1.0/9.4)
            .withKP(2.0)
            .withKI(0.05)
            .withKD(0.3);
        setLeftConfig(conf);
        MOTOR_RIGHT.setControl(
            new Follower(Constants.Device.ELEVATOR_LEFT.ID, true)
        );
        this.CONTROL = new PositionVoltage(0.0)
            .withSlot(0)
            .withEnableFOC(true);
        this.FEED_CTRL = new ElevatorFeedforward(0.4, 0.16, 1.0/9.4);

        // <<< EDIT: use base constraints from Constants instead of hardcoded 50/100 >>>
        this.PROFILE = new TrapezoidProfile(
            new Constraints(
                Constants.ELEVATOR_BASE_MAX_VEL_ROT_PER_S,
                Constants.ELEVATOR_BASE_MAX_ACC_ROT_PER_S2
            )
        );

        this.currentState = new State();
        this.targetState = new State();
        this.END_EFFECTOR = new KrakenX60(Constants.Device.ELEVATOR_END_EFFECTOR.ID);
        this.CORAL_INTAKE = new KrakenX60(Constants.Device.ELEVATOR_CORAL_INTAKE.ID);
        this.PROXIMITY_SENSORS = new CANrange[]{
            new CANrange(Constants.Device.ELEVATOR_INTAKE_PROX.ID),
            new CANrange(Constants.Device.ELEVATOR_END_EFFECTOR_PROX_IN.ID),
            new CANrange(Constants.Device.ELEVATOR_END_EFFECTOR_PROX_OUT.ID)
        };
        this.heightOffset = 0;
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        TalonFXConfiguration endEffectorConfig = new TalonFXConfiguration();
        endEffectorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        endEffectorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        CORAL_INTAKE.getConfigurator().apply(intakeConfig);
        END_EFFECTOR.getConfigurator().apply(endEffectorConfig);
    }

    /**
     * sets the height of the elevator
     * @param height elevator height (in)
     */
    public void setHeight(double height) {
        height += heightOffset;
        setFromRotations(getRotationsFromDistance(height));
    }

    public void zero() {
        MOTOR_LEFT.setPosition(0);
    }

    public boolean isCoralInIntake() {
        return getProximitySensorValue(0) < 0.1;
    }

    public boolean isCoralInTransit() {
        return getProximitySensorValue(1) < 0.2;
    }

    public boolean isCoralInEndEffector() {
        return getProximitySensorValue(2) < 0.2;
    }

    public double getProximitySensorValue(int sensorId) {
        return PROXIMITY_SENSORS[sensorId].getDistance().getValueAsDouble();
    }

    /**
     * sets config of the motor duhh
     * @param config motor config object
     */
    private void setLeftConfig(TalonFXConfiguration config) {
        MOTOR_LEFT.getConfigurator().apply(config);
    }

    public void setHeightOffset(double offset) {
        heightOffset = offset;
    }

    public double getHeightOffset() {
        return heightOffset;
    }

    private static double getDistanceFromRotations(double rotations) {
        return (CIRCUMFERENCE / 4.8) * rotations;
    }

    /**
     * sets target position
     * @param position target position
     */
    public void setFromRotations(double position) {
        currentState = new State(
            MOTOR_LEFT.getPosition().getValueAsDouble(), 
            MOTOR_LEFT.getVelocity().getValueAsDouble()
        );
        targetState = new State(position, 0.0);
    }

    public double getRotations() {
        return this.MOTOR_LEFT.getPosition().getValueAsDouble();
    }

    public double getHeight() {
        return getDistanceFromRotations(getRotations());   
    }

    /**
     * tells the elevator it should shoot coral 
     * (doesnt do anythng if theres no coral detected)
     */
    public void ejectCoral() {
        shouldShoot = true;
    }

    public void setEjection(boolean state) {
        shouldShoot = state;
    }

    public boolean isEjecting() {
        return shouldShoot;
    }

    public void setAlgaeMode(boolean mode) {
        intakeAlgae = mode;
    }

    public boolean inAlgaeMode() {
        return intakeAlgae;
    }

    private static double getRotationsFromDistance(double distance) {
        return distance / (CIRCUMFERENCE / 4.8);
    }

    // helper to check if target equals a specific preset (inches)
    private boolean isAtHeightInches(double inches) {
        return RobotMath.fEquals(targetState.position, getRotationsFromDistance(inches));
    }

    // <<< NEW: L4+offset target height (in) and equality test on targetState >>>
    private double getAlgaeL4HeightIn() {
        return Constants.ELEVATOR_HEIGHTS[4] + Constants.ALGAE_L4_OFFSET_IN;
    }
    private boolean isTargetL4Algae() {
        return RobotMath.fEquals(
            targetState.position,
            getRotationsFromDistance(getAlgaeL4HeightIn())
        );
    }

    // existing L1-only tunable accessor (+ clamp)
    private double getShootPowerL1() {
        double raw = SmartDashboard.getNumber("Elevator/ShootPowerL1", DEFAULT_SHOOT_POWER_L1);
        return MathUtil.clamp(raw, -1.0, 1.0);
    }

    @Override
    public void start() {
        zero();
        setHeight(0);
        // seed DS tunables
        SmartDashboard.putNumber("Elevator/ShootPowerL1", DEFAULT_SHOOT_POWER_L1);
        // NEW: seed global power scale so autos/teleop can control it
        SmartDashboard.putNumber(SHOT_SCALE_KEY, DEFAULT_SHOT_POWER_SCALE);
    }

    @Override
    public void update() {

        // ====== PRE-FIRE WHILE RISING (no settle) — only when targeting L4+offset in algae mode ======
        if (intakeAlgae && isTargetL4Algae()) {
            double remainingIn = getAlgaeL4HeightIn() - getHeight(); // >0 while below top
            if (remainingIn <= Constants.ALGAE_L4_PREFIRE_WINDOW_IN) {
                if (!shouldShoot) {
                    shouldShoot = true;              // start shooting while still moving up
                    algaeFireTimer.stop();           // clean
                    algaeFireTimer.reset();
                    algaeFireTimer.start();          // bound shot time
                }
            }
            // Stop the shot after configured duration
            if (shouldShoot && algaeFireTimer.get() >= Constants.ALGAE_L4_SHOOT_TIME_S) {
                shouldShoot = false;
                algaeFireTimer.stop();
                algaeFireTimer.reset();
            }
        } else {
            // Not in the Algae L4 one-shot context: just reset the timer; do NOT force shouldShoot false
            algaeFireTimer.stop();
            algaeFireTimer.reset();
        }
        // ====== END pre-fire block ======

        if (!intakeAlgae) {
            if (isCoralInIntake()) {
                CORAL_INTAKE.set(0.2);
                END_EFFECTOR.set(0.15);
            } else if (isCoralInTransit()) {
                END_EFFECTOR.set(0.1);
                CORAL_INTAKE.set(0.0);
            } else if (isCoralInEndEffector()) {
                //special case
                if (shouldShoot) {
                    // Base power normally 0.75, or use L1 DS tunable when at L1
                    double power = 0.75;
                    if (isAtHeightInches(Constants.ELEVATOR_HEIGHTS[1])) {
                        power = getShootPowerL1();
                    }
                    // NEW: apply global shot-power scale (e.g., L2 auto sets < 1.0)
                    power = MathUtil.clamp(power * getShotPowerScale(), -1.0, 1.0);
                    END_EFFECTOR.set(power);

                    if (RobotMath.fEquals(targetState.position, getRotationsFromDistance(Constants.ELEVATOR_HEIGHTS[4]))) {
                        setHeight(Constants.ELEVATOR_HEIGHTS[4] + 2.5);
                    }
                } else {
                    CORAL_INTAKE.set(0.0);
                    END_EFFECTOR.set(0.0);
                }
            } else {
                END_EFFECTOR.set(0.0);
                CORAL_INTAKE.set(0.1);
                if (shouldShoot) {
                    //horror
                    double orgRot = getRotations(); //eff. final original height
                    AsyncComputeTask<Boolean> delay = new AsyncComputeTask<>(() -> {
                        try {
                            Thread.sleep(1000);
                        } catch (Exception ex) {
                            System.err.println("error in elevator delay");
                        }
                        return !shouldShoot && 
                            RobotMath.fEquals(
                                targetState.position, 
                                orgRot
                            ); //cuz i thnk it shouldnt go down if the elev. was moved by the op
                    });
                    delay.onCompletion().register((shouldGoDown, connection) -> {
                        if (shouldGoDown) {
                            setHeight(Constants.ELEVATOR_HEIGHTS[0]);
                        }
                        connection.disconnect();
                    });
                    delay.compute();
                }
                shouldShoot = false;
            }
        } else {
            // algae mode unchanged (shooting behavior governed by shouldShoot from pre-fire block)
            CORAL_INTAKE.set(0.0);
            if (shouldShoot) {
                END_EFFECTOR.set(1.0);
            } else if (!isCoralInEndEffector()) {
                END_EFFECTOR.set(-0.5);
            } else {
                END_EFFECTOR.set(-0.2);
            }
        }

        // <<< EDIT: Dynamic constraints — use faster profile ONLY for Algae L4 moves >>>
        TrapezoidProfile activeProfile = (intakeAlgae && isTargetL4Algae())
            ? new TrapezoidProfile(new Constraints(
                    Constants.ALGAE_L4_MAX_VEL_ROT_PER_S,
                    Constants.ALGAE_L4_MAX_ACC_ROT_PER_S2))
            : PROFILE;

        State nextState = activeProfile.calculate(0.02, currentState, targetState);

        MOTOR_LEFT.setControl(
            CONTROL.withPosition(nextState.position)
                .withFeedForward(FEED_CTRL
                    .calculateWithVelocities(currentState.velocity, nextState.velocity)
                )
        );
        currentState = nextState;
    }

    @Override
    public void stop() {
        setHeight(getHeight());
    }
}
