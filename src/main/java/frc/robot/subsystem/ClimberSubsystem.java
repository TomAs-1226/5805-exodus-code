package frc.robot.subsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  // ==================== CAN + IDs ====================
  // Use "*" to bind to first CANivore; replace with your explicit name if you prefer.
  private static final String CAN_BUS = "*";
  private static final int CLIMBER_H_ID  = 52; // Kraken X44 (intake)
  private static final int CLIMBER_GB_ID = 50; // Kraken X60 (GB/boom)

  // ==================== PID / behavior ====================
  public static final double GB_KP = 20.0, GB_KI = 0.0, GB_KD = 0.0, GB_KS = 0.0, GB_KV = 0.0, GB_KA = 0.0;
  public static final double GB_TOLERANCE_DEG = 2.0;        // “at target” (deg)
  public static final double INTAKE_SPEED = -0.5;          // H motor (%)
  public static final double STALL_SUPPLY_CURRENT_A = 38.0; // legacy (kept if you still need it)
  public static final double STALL_DEBOUNCE_S = 0.25;

  // ==================== Hardware ====================
  private final TalonFX climberH  = new TalonFX(CLIMBER_H_ID,  CAN_BUS);
  private final TalonFX climberGB = new TalonFX(CLIMBER_GB_ID, CAN_BUS);

  // Closed-loop position (Phoenix 6 expects rotations)
  private final PositionVoltage gbPosCtrl = new PositionVoltage(0.0).withSlot(0);

  // Open-loop request (reused)
  private final DutyCycleOut gbDuty = new DutyCycleOut(0.0);

  // ==================== Voltage detection signals (fast) ====================
  private final StatusSignal<Voltage> gbSupplyV = climberGB.getSupplyVoltage(); // battery sag at the device
  private final StatusSignal<Voltage> gbMotorV  = climberGB.getMotorVoltage();  // applied voltage at output

  // ==================== Voltage detection tunables (SmartDashboard) ====================
  // (All can be changed live on the dashboard.)
  private static final double DEF_VDROP_V          = 0.50;  // absolute drop (V) to trip
  private static final double DEF_DVDT_V_PER_S     = 2.00;  // rate-of-drop (V/s) to trip
  private static final double DEF_DEBOUNCE_S       = 0.12;  // require condition to hold this long
  private static final boolean DEF_USE_DEBOUNCE    = true;  // enable/disable timer
  private static final boolean DEF_VCONTACT_ENABLE = true;  // master enable for voltage detection
  private static final double DEF_BASELINE_ALPHA   = 0.02;  // 0..1, how fast baseline follows (smaller = steadier)

  // Runtime copies
  private boolean vDetectEnabled   = DEF_VCONTACT_ENABLE;
  private boolean vUseDebounce     = DEF_USE_DEBOUNCE;
  private double  vDropThreshV     = DEF_VDROP_V;
  private double  vDvdtThreshVperS = DEF_DVDT_V_PER_S;
  private double  vDebounceS       = DEF_DEBOUNCE_S;
  private double  baselineAlpha    = DEF_BASELINE_ALPHA;

  // State for detection
  private double baselineV = 12.0;
  private double lastV     = 12.0;
  private double lastT     = Timer.getFPGATimestamp();
  private double dvdt      = 0.0;
  private double dropV     = 0.0;
  private double debounceStart = -1.0;
  private boolean voltageContactLatched = false;

  public ClimberSubsystem() {
    // ===== GB motor config =====
    TalonFXConfigurator gbCfg = climberGB.getConfigurator();
    TalonFXConfiguration gb = new TalonFXConfiguration();

    gb.Slot0.kP = GB_KP; gb.Slot0.kI = GB_KI; gb.Slot0.kD = GB_KD;
    gb.Slot0.kS = GB_KS; gb.Slot0.kV = GB_KV; gb.Slot0.kA = GB_KA;

    CurrentLimitsConfigs gbCurrent = new CurrentLimitsConfigs();
    gbCurrent.SupplyCurrentLimitEnable = true;
    gbCurrent.SupplyCurrentLimit = 40.0;
    gb.CurrentLimits = gbCurrent;

    gb.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Keep soft-limits off during homing; you can re-enable after calibration if desired
    SoftwareLimitSwitchConfigs soft = new SoftwareLimitSwitchConfigs();
    soft.ForwardSoftLimitEnable = false;
    soft.ReverseSoftLimitEnable = false;
    gb.SoftwareLimitSwitch = soft;

    gbCfg.apply(gb);

    // ===== H motor config =====
    TalonFXConfigurator hCfg = climberH.getConfigurator();
    TalonFXConfiguration h = new TalonFXConfiguration();

    CurrentLimitsConfigs hCurrent = new CurrentLimitsConfigs();
    hCurrent.SupplyCurrentLimitEnable = true;
    hCurrent.SupplyCurrentLimit = 35.0;
    h.CurrentLimits = hCurrent;

    h.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hCfg.apply(h);

    // ===== Fast signal rates for detection =====
    gbSupplyV.setUpdateFrequency(100); // 100 Hz for sensitivity
    gbMotorV.setUpdateFrequency(100);  // optional telemetry
    // (You can also refresh multiple at once; see periodic())

    // ===== Seed dashboard tunables =====
    SmartDashboard.putBoolean("Climb/VDetectEnabled",    DEF_VCONTACT_ENABLE);
    SmartDashboard.putBoolean("Climb/VUseDebounce",      DEF_USE_DEBOUNCE);
    SmartDashboard.putNumber ("Climb/VDropV",            DEF_VDROP_V);
    SmartDashboard.putNumber ("Climb/VDvdtVperS",        DEF_DVDT_V_PER_S);
    SmartDashboard.putNumber ("Climb/VDebounceS",        DEF_DEBOUNCE_S);
    SmartDashboard.putNumber ("Climb/VBaselineAlpha",    DEF_BASELINE_ALPHA);
  }

  // ==================== Degrees <-> rotations ====================
  private static double degToRot(double deg) { return deg / 360.0; }
  private static double rotToDeg(double rot) { return rot * 360.0; }

  // ==================== GB position helpers ====================
  public void commandGBToDegrees(double degrees) {
    climberGB.setControl(gbPosCtrl.withPosition(degToRot(degrees)));
  }
  public boolean isGBAtDegrees(double degrees) {
    return Math.abs(getGBPositionDegrees() - degrees) <= GB_TOLERANCE_DEG;
  }
  public double getGBPositionDegrees() {
    return rotToDeg(climberGB.getPosition().refresh().getValueAsDouble());
  }
  /** After homing/backoff, set the mechanism position to a calibrated angle. Returns true on success. */
  public boolean calibrateGBPositionDegrees(double degrees) {
    StatusCode sc = climberGB.setPosition(degToRot(degrees), 0.25); // wait up to 250ms
    return sc.isOK();
  }

  // ==================== GB open-loop (homing/jog) ====================
  public void runGBPercent(double percent) { climberGB.setControl(gbDuty.withOutput(percent)); }
  public void stopGB() { climberGB.stopMotor(); }

  // ==================== Currents (legacy helpers you already used) ====================
  public double getGBSupplyCurrent() { return climberGB.getSupplyCurrent().refresh().getValueAsDouble(); }
  public double getGBStatorCurrent() { return climberGB.getStatorCurrent().refresh().getValueAsDouble(); }

  // ==================== H intake ====================
  public void runHPercent(double percent) { climberH.setControl(new DutyCycleOut(percent)); }
  public void stopH() { climberH.stopMotor(); }
  public double getHSupplyCurrent() { return climberH.getSupplyCurrent().refresh().getValueAsDouble(); }

  // ==================== Soft limits ====================
  public void disableSoftLimits() {
    SoftwareLimitSwitchConfigs soft = new SoftwareLimitSwitchConfigs();
    soft.ForwardSoftLimitEnable = false;
    soft.ReverseSoftLimitEnable = false;
    climberGB.getConfigurator().apply(soft);
  }
  public void applySoftLimitsDegrees(Double reverseMinDeg, Double forwardMaxDeg) {
    SoftwareLimitSwitchConfigs soft = new SoftwareLimitSwitchConfigs();
    if (reverseMinDeg != null) {
      soft.ReverseSoftLimitEnable = true;
      soft.ReverseSoftLimitThreshold = degToRot(reverseMinDeg);
    }
    if (forwardMaxDeg != null) {
      soft.ForwardSoftLimitEnable = true;
      soft.ForwardSoftLimitThreshold = degToRot(forwardMaxDeg);
    }
    climberGB.getConfigurator().apply(soft);
  }

  public void stopAll() { stopH(); stopGB(); }

  // ==================== Voltage-contact detection ====================
  /**
   * More-sensitive “voltage spike” detector that trips when either:
   *  1) Supply voltage drop from a rolling baseline exceeds VDrop (volts), OR
   *  2) Rate-of-drop dV/dt exceeds Dvdt (V/s).
   * Optionally requires the condition to hold >= DebounceS when enabled.
   *
   * Tune on SmartDashboard:
   *  - Climb/VDetectEnabled (bool)
   *  - Climb/VUseDebounce (bool)
   *  - Climb/VDropV (double)
   *  - Climb/VDvdtVperS (double)
   *  - Climb/VDebounceS (double)
   *  - Climb/VBaselineAlpha (double 0..1)
   */
  public boolean isVoltageContact() { return voltageContactLatched; }

  /** Live thresholds control from your Robot if you prefer code-side tuning. */
  public void setVoltageDetectParams(boolean enabled, boolean useDebounce,
                                     double dropV, double dvdtVperS,
                                     double debounceS, double baselineAlpha_) {
    vDetectEnabled   = enabled;
    vUseDebounce     = useDebounce;
    vDropThreshV     = dropV;
    vDvdtThreshVperS = dvdtVperS;
    vDebounceS       = debounceS;
    baselineAlpha    = clamp(baselineAlpha_, 0.001, 1.0);
  }

  /** Telemetry helpers (optional). */
  public double getSupplyVoltage() { return gbSupplyV.refresh().getValueAsDouble(); }
  public double getMotorVoltage()  { return gbMotorV.refresh().getValueAsDouble(); }
  public double getVoltageDropV()  { return dropV; }
  public double getVoltageDvdt()   { return dvdt; }

  @Override
  public void periodic() {
    // Pull live tunables (so you can edit on glass without redeploying)
    vDetectEnabled   = SmartDashboard.getBoolean("Climb/VDetectEnabled",  DEF_VCONTACT_ENABLE);
    vUseDebounce     = SmartDashboard.getBoolean("Climb/VUseDebounce",    DEF_USE_DEBOUNCE);
    vDropThreshV     = SmartDashboard.getNumber ("Climb/VDropV",          DEF_VDROP_V);
    vDvdtThreshVperS = SmartDashboard.getNumber ("Climb/VDvdtVperS",      DEF_DVDT_V_PER_S);
    vDebounceS       = SmartDashboard.getNumber ("Climb/VDebounceS",      DEF_DEBOUNCE_S);
    baselineAlpha    = SmartDashboard.getNumber ("Climb/VBaselineAlpha",  DEF_BASELINE_ALPHA);
    baselineAlpha    = clamp(baselineAlpha, 0.001, 1.0);

    // Refresh signals together for consistency and lower bus use
    BaseStatusSignal.refreshAll(gbSupplyV, gbMotorV);

    final double now = Timer.getFPGATimestamp();
    final double v   = gbSupplyV.getValueAsDouble();
    final double dt  = Math.max(1e-3, now - lastT);

    // Update baseline (EWMA). Tracks slowly so brief sags stand out.
    baselineV = (1.0 - baselineAlpha) * baselineV + baselineAlpha * v;

    // Compute metrics
    dropV = Math.max(0.0, baselineV - v);      // absolute drop
    dvdt  = Math.max(0.0, (lastV - v) / dt);   // positive when voltage is falling

    // Trip condition
    boolean rawTrip = vDetectEnabled && (dropV >= vDropThreshV || dvdt >= vDvdtThreshVperS);

    if (!vUseDebounce) {
      voltageContactLatched = rawTrip;
    } else {
      if (rawTrip) {
        if (debounceStart < 0) debounceStart = now;
        voltageContactLatched = (now - debounceStart) >= vDebounceS;
      } else {
        debounceStart = -1.0;
        voltageContactLatched = false;
      }
    }

    // Telemetry (handy during tuning)
    SmartDashboard.putNumber("Climb/VNow", v);
    SmartDashboard.putNumber("Climb/VBase", baselineV);
    SmartDashboard.putNumber("Climb/VDrop", dropV);
    SmartDashboard.putNumber("Climb/VdVdt", dvdt);
    SmartDashboard.putBoolean("Climb/VContact", voltageContactLatched);

    // Update history
    lastV = v;
    lastT = now;
  }

  // ==================== utils ====================
  private static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }
}
