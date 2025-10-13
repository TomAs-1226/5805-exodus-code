package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.command.CommandSystem;
import frc.robot.subsystem.SubsystemManager;
import frc.robot.subsystem.drive.SwerveDrivetrain;
import frc.robot.subsystem.elevator.Elevator;
import frc.robot.subsystem.ClimberSubsystem;

import frc.robot.vision.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// shaping/smoothing
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

// PID for heading hold
import edu.wpi.first.math.controller.PIDController;

// chooser on Shuffleboard (already in your project)
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

// Pigeon2 for yaw (heading hold)
import com.ctre.phoenix6.hardware.Pigeon2;

public class Robot extends TimedRobot {

  private final CommandSystem COMSYS;
  private final Elevator ELEVATOR;
  private final SwerveDrivetrain DRIVETRAIN;
  private final ClimberSubsystem CLIMBER;

  private static final PS5Controller CONTROLLER = new PS5Controller(0);
  private boolean foc = true;
  private boolean autoaim = false;

  // ===== Driver feel =====
  private static final double TRANS_DEADBAND = 0.05;
  private static final double ROT_DEADBAND   = 0.06;
  private static final double TRANS_EXPO = 2.0;
  private static final double ROT_EXPO   = 2.4;
  private static final double TRANS_SLEW = 3.0;
  private static final double ROT_SLEW   = 2.0;
  private static final double ROT_GAIN   = 0.60;

  private final SlewRateLimiter xLimiter   = new SlewRateLimiter(TRANS_SLEW);
  private final SlewRateLimiter yLimiter   = new SlewRateLimiter(TRANS_SLEW);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(ROT_SLEW);

  // ===== Climb tunables =====
  private static final double DEFAULT_ZERO_OFFSET_DEG      = 0.0;
  private static final double DEFAULT_PARK_AFTER_ZERO_DEG  = 45.0;
  private static final double DEFAULT_HOME_STALL_A         = 14.0;
  private static final double DEFAULT_HOME_DEBOUNCE_S      = 0.20;
  private static final double DEFAULT_HOME_BACKOFF_DEG     = 5.0;
  private static final double DEFAULT_HOME_SPEED_MAG       = 0.08;

  // >>> FASTER manual jog defaults
  private static final double DEFAULT_MANUAL_JOG_PERCENT   = 0.14; // base duty (0..1)
  private static final double DEFAULT_MANUAL_JOG_OUT_MULT  = 2.6;  // OUTWARD boost (>=1)
  private static final double DEFAULT_MANUAL_JOG_IN_MULT   = 1.8;  // INWARD  boost (>=1)

  private static final double DEFAULT_H_TOGGLE_PERCENT = -0.25;
  private boolean hToggleActive = false;
  private double getHTogglePercent() {
    return SmartDashboard.getNumber("Climb/HTogglePercent", DEFAULT_H_TOGGLE_PERCENT);
  }

  private static final double PROBE_TIME_S           = 0.35;
  private static final double PROBE_MIN_TRAVEL_DEG   = 10.0;

  private double getZeroOffsetDeg()     { return SmartDashboard.getNumber("Climb/ZeroOffsetDeg",     DEFAULT_ZERO_OFFSET_DEG); }
  private double getParkAfterZeroDeg()  { return SmartDashboard.getNumber("Climb/ParkAfterZeroDeg",  DEFAULT_PARK_AFTER_ZERO_DEG); }
  private double getHomeStallA()        { return SmartDashboard.getNumber("Climb/HomeStallA",        DEFAULT_HOME_STALL_A); }
  private double getHomeDebounceS()     { return SmartDashboard.getNumber("Climb/HomeDebounceS",     DEFAULT_HOME_DEBOUNCE_S); }
  private double getHomeBackoffDeg()    { return SmartDashboard.getNumber("Climb/HomeBackoffDeg",    DEFAULT_HOME_BACKOFF_DEG); }
  private double getHomeSpeedMag()      { return SmartDashboard.getNumber("Climb/HomeSpeedMag",      DEFAULT_HOME_SPEED_MAG); }

  private double getManualJogPercent()  { return SmartDashboard.getNumber("Climb/ManualJogPercent",  DEFAULT_MANUAL_JOG_PERCENT); }
  private double getManualJogOutMult()  { return SmartDashboard.getNumber("Climb/ManualJogOutMult",  DEFAULT_MANUAL_JOG_OUT_MULT); }
  private double getManualJogInMult()   { return SmartDashboard.getNumber("Climb/ManualJogInMult",   DEFAULT_MANUAL_JOG_IN_MULT); }

  // ===== Auto chooser =====
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private String autoSelected = "do_nothing";

  // ===== Auto: distance-based tunables (SmartDashboard) =====
  private static final double DEFAULT_IPS_PER_CMD          = 120.0; // inches/sec at cmd=1.0 (CALIBRATE THIS)
  private static final double DEFAULT_MOVE_CMD             = 0.35;  // forward command (0..1). Sign sets forward direction.
  private static final double DEFAULT_RAISE_DELAY_S        = 0.25;  // used by L3

  private static final double DEFAULT_TARGET_DIST_L2_IN    = 40.0;  
  private static final double DEFAULT_BACKOFF_L2_IN        = 7.0;

  private static final double DEFAULT_TARGET_DIST_L3_IN    = 80.0;
  private static final double DEFAULT_TARGET_DIST_L4_IN    = 82.0;
  private static final double DEFAULT_BACKOFF_L4_IN        = 4.0;   // retreat before raising to L4 (helps avoid barge)

  private static final double DEFAULT_SHOT_SCALE_L2        = 0.85;  // slightly reduced power for L2

  // Simple Leave (just drive forward)
  private static final double DEFAULT_LEAVE_DIST_IN        = 44.0;

  private static final double EJECT_TIME_S                 = 0.60;  // common eject duration
  private static final double L2_TOL_IN                    = 1.0;
  private static final double L3_TOL_IN                    = 1.0;
  private static final double L4_TOL_IN                    = 1.0;
  private static final double L2_RAISE_TIMEOUT_S           = 2.0;
  private static final double L3_RAISE_TIMEOUT_S           = 2.5;
  private static final double L4_RAISE_TIMEOUT_S           = 3.0;

  private double getIPSPerCmd()      { return SmartDashboard.getNumber("Auto/IPSPerCmd",   DEFAULT_IPS_PER_CMD); }
  private double getMoveCmd()        { return SmartDashboard.getNumber("Auto/MoveCmd",     DEFAULT_MOVE_CMD); }
  private double getRaiseDelayS()    { return SmartDashboard.getNumber("Auto/RaiseDelayS", DEFAULT_RAISE_DELAY_S); }

  private double getTargetL2In()     { return SmartDashboard.getNumber("Auto/TargetDistL2In", DEFAULT_TARGET_DIST_L2_IN); }
  private double getBackoffL2In()    { return SmartDashboard.getNumber("Auto/BackoffL2In",   DEFAULT_BACKOFF_L2_IN); }
  private double getShotScaleL2()    { return SmartDashboard.getNumber("Auto/ShotPowerScaleL2", DEFAULT_SHOT_SCALE_L2); }

  private double getTargetL3In()     { return SmartDashboard.getNumber("Auto/TargetDistL3In", DEFAULT_TARGET_DIST_L3_IN); }
  private double getTargetL4In()     { return SmartDashboard.getNumber("Auto/TargetDistL4In", DEFAULT_TARGET_DIST_L4_IN); }
  private double getBackoffL4In()    { return SmartDashboard.getNumber("Auto/BackoffL4In",   DEFAULT_BACKOFF_L4_IN); }

  private double getLeaveDistIn()    { return SmartDashboard.getNumber("Auto/LeaveDistIn",   DEFAULT_LEAVE_DIST_IN); }

  // ===== Heading-hold (gyro-based) to prevent skew =====
  private static final double HEAD_KP = 0.02; // tune on carpet
  private static final double HEAD_KI = 0.00;
  private static final double HEAD_KD = 0.001;

  private final PIDController headingPid = new PIDController(HEAD_KP, HEAD_KI, HEAD_KD);
  private double headingTargetDeg = 0.0;
  private boolean headingLocked = false;
  private final Pigeon2 IMU = new Pigeon2(Constants.Device.PIGEON_2.ID);

  private void headingHoldStart() {
    headingTargetDeg = IMU.getYaw().getValueAsDouble(); // degrees
    headingPid.reset();
    headingPid.enableContinuousInput(-180.0, 180.0);
    headingLocked = true;
  }
  private void headingHoldStop() {
    headingLocked = false;
  }
  private double headingHoldOmega() {
    if (!headingLocked) return 0.0;
    double yawDeg = IMU.getYaw().getValueAsDouble();
    double cmd = headingPid.calculate(yawDeg, headingTargetDeg);
    return MathUtil.clamp(cmd, -1.0, 1.0);
  }

  // ===== Simple autonomous runner =====
  private enum AutoState { INIT, RAISE_FIRST, MOVE_FWD, WAIT_FOR_HEIGHT, BACKOFF, EJECT, DONE }
  private AutoState autoState = AutoState.INIT;

  private final Timer autoTimer = new Timer();          // generic step timer
  private final Timer raiseDelayTimer = new Timer();    // tiny delay for L3

  // --- Simple "open-loop odometry" along field +Y (inches) ---
  private double odomYIn = 0.0;       // integrated distance (in), + forward
  private double segStartYIn = 0.0;   // segment start for forward drive
  private double segStartYInBack = 0.0; // segment start for backoff
  private double lastTSec = 0.0;      // last timestamp
  private double currentFwdCmd = 0.0; // command we integrate each loop
  private boolean raiseIssued = false;

  // Only orient once at the very start of the match
  private boolean orientedForMatch = false;

  // ===== Climb SM (unchanged) =====
  private enum Phase { IDLE, PROBE, HOME_PUSH, HOME_BACKOFF, CALIBRATE_AND_PARK }
  private Phase phase = Phase.IDLE;
  private final Timer timer = new Timer();
  private double debounceStart = -1.0;
  private int lastPOV = -1;

  private double homeDirSign = +1.0;
  private boolean homeDirLearned = false;
  private double probeStartDeg = 0.0;

  public Robot() {
    this.COMSYS     = new CommandSystem(this);
    this.ELEVATOR   = SubsystemManager.registerSubsystem(Elevator::new);
    this.DRIVETRAIN = SubsystemManager.registerSubsystem(SwerveDrivetrain::new);
    this.CLIMBER    = new ClimberSubsystem();
    SubsystemManager.start();
  }

  @Override
  public void robotInit() {
    // Keep: Do Nothing, Leave, Score L2 (backoff), Score L3, Score L4 (safe)
    autoChooser.setDefaultOption("Do Nothing", "do_nothing");
    autoChooser.addOption("Leave", "leave");
    autoChooser.addOption("Score L2 (backoff 6\")", "score_l2_backoff6");
    autoChooser.addOption("Score L3", "score_l3");
    autoChooser.addOption("Score L4 (safe L2-first)", "score_l4_safe");

    SmartDashboard.putData("Auto Selector", autoChooser);
    Shuffleboard.getTab("Autonomous")
      .add("Auto Selector", autoChooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser);

    // Seed auto tunables
    SmartDashboard.putNumber("Auto/IPSPerCmd",   DEFAULT_IPS_PER_CMD);
    SmartDashboard.putNumber("Auto/MoveCmd",     DEFAULT_MOVE_CMD);
    SmartDashboard.putNumber("Auto/RaiseDelayS", DEFAULT_RAISE_DELAY_S);

    SmartDashboard.putNumber("Auto/TargetDistL2In", DEFAULT_TARGET_DIST_L2_IN);
    SmartDashboard.putNumber("Auto/BackoffL2In",    DEFAULT_BACKOFF_L2_IN);
    SmartDashboard.putNumber("Auto/ShotPowerScaleL2", DEFAULT_SHOT_SCALE_L2);

    SmartDashboard.putNumber("Auto/TargetDistL3In", DEFAULT_TARGET_DIST_L3_IN);
    SmartDashboard.putNumber("Auto/TargetDistL4In", DEFAULT_TARGET_DIST_L4_IN);
    SmartDashboard.putNumber("Auto/BackoffL4In",    DEFAULT_BACKOFF_L4_IN);

    SmartDashboard.putNumber("Auto/LeaveDistIn",    DEFAULT_LEAVE_DIST_IN);

    SmartDashboard.putNumber("Auto/OdomYIn", 0.0);
  }

  @Override public void robotPeriodic() { SubsystemManager.update(); }

  @Override
  public void autonomousInit() {
    autoSelected = autoChooser.getSelected();
    System.out.println("[Auto] Selected: " + autoSelected);

    DRIVETRAIN.start();
    if (!orientedForMatch) {
      DRIVETRAIN.orientFacingDriverStation(); // set yaw = 180 so field +Y is "toward reef"
      orientedForMatch = true;
    }
    DRIVETRAIN.setDriveMaxAll(0.60);
    DRIVETRAIN.setSteerMaxAll(0.85);

    // reset simple odom
    odomYIn = 0.0;
    segStartYIn = 0.0;
    segStartYInBack = 0.0;
    currentFwdCmd = 0.0;
    lastTSec = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Auto/OdomYIn", odomYIn);

    // timers
    autoTimer.stop(); autoTimer.reset();
    raiseIssued = false;
    raiseDelayTimer.stop(); raiseDelayTimer.reset();

    // Program the shot power scale only for the L2 auto; reset otherwise
    if ("score_l2_backoff6".equals(autoSelected)) {
      SmartDashboard.putNumber("Elevator/ShotPowerScale", getShotScaleL2());
    } else {
      SmartDashboard.putNumber("Elevator/ShotPowerScale", 1.0);
    }

    headingHoldStop(); // reset
    autoState = AutoState.INIT;
  }

  @Override
  public void autonomousPeriodic() {
    // --- integrate open-loop odometry along field +Y each loop ---
    double now = Timer.getFPGATimestamp();
    double dt  = now - lastTSec;
    lastTSec   = now;
    odomYIn   += currentFwdCmd * getIPSPerCmd() * dt; // ips = cmd * IPSPerCmd
    SmartDashboard.putNumber("Auto/OdomYIn", odomYIn);

    // default to no integration unless a state sets it
    currentFwdCmd = 0.0;

    switch (autoSelected) {
      case "leave":
        runLeaveForward();
        break;
      case "score_l2_backoff6":
        runScoreL2_RaiseThenGo_Eject_Backoff6();
        break;
      case "score_l3":
        runScore_Lx(Constants.ELEVATOR_HEIGHTS[3], getTargetL3In(), L3_TOL_IN, L3_RAISE_TIMEOUT_S);
        break;
      case "score_l4_safe":
        runScore_L4_Safe();  // << updated sequence you requested
        break;
      default:
        DRIVETRAIN.stop();
        break;
    }
  }

  // ===== Simple Leave: heading-hold straight, drive forward set distance, stop =====
  private void runLeaveForward() {
    final double driveTargetIn = getLeaveDistIn();

    switch (autoState) {
      case INIT: {
        DRIVETRAIN.stop();
        segStartYIn = odomYIn;
        headingHoldStart();                 // lock current heading to prevent skew
        autoState = AutoState.MOVE_FWD;
        System.out.println("[Leave] INIT → MOVE_FWD (targetDist=" + driveTargetIn + " in)");
        break;
      }
      case MOVE_FWD: {
        currentFwdCmd = getMoveCmd();
        double omega = headingHoldOmega();
        DRIVETRAIN.drive(0.0, currentFwdCmd, omega, true);

        double distThisSeg = Math.abs(odomYIn - segStartYIn);
        if (distThisSeg >= Math.abs(driveTargetIn)) {
          DRIVETRAIN.stop();
          headingHoldStop();
          autoState = AutoState.DONE;
          System.out.println("[Leave] MOVE_FWD complete (" + distThisSeg + " in) → DONE");
        }
        break;
      }
      default: {
        DRIVETRAIN.stop();
        break;
      }
    }
  }

  // ===== L2: raise first -> drive -> eject -> backoff ~6" (with heading hold on straight segments) =====
  private void runScoreL2_RaiseThenGo_Eject_Backoff6() {
    final double targetHeightIn = Constants.ELEVATOR_HEIGHTS[2]; // L2
    final double driveTargetIn  = getTargetL2In();
    final double backoffIn      = getBackoffL2In();

    switch (autoState) {
      case INIT: {
        DRIVETRAIN.stop();
        // Raise immediately before moving
        ELEVATOR.setHeight(targetHeightIn);
        autoTimer.reset(); autoTimer.start();
        autoState = AutoState.RAISE_FIRST;
        System.out.println("[Auto L2] INIT → RAISE_FIRST (height=" + targetHeightIn + " in)");
        break;
      }

      case RAISE_FIRST: {
        boolean atHeight = ELEVATOR.atHeightInches(targetHeightIn, L2_TOL_IN);
        boolean timeout  = autoTimer.get() > L2_RAISE_TIMEOUT_S;
        if (atHeight || timeout) {
          if (timeout) System.out.println("[Auto L2] RAISE_FIRST timeout—continuing");
          segStartYIn = odomYIn;                 // start distance segment now
          headingHoldStart();                    // lock heading before driving
          autoTimer.reset(); autoTimer.start();
          autoState = AutoState.MOVE_FWD;
          System.out.println("[Auto L2] RAISE_FIRST → MOVE_FWD (targetDist=" + driveTargetIn + " in)");
        }
        break;
      }

      case MOVE_FWD: {
        // drive forward to distance with heading hold
        currentFwdCmd = getMoveCmd();
        double omega = headingHoldOmega();
        DRIVETRAIN.drive(0.0, currentFwdCmd, omega, true);

        double distThisSeg = Math.abs(odomYIn - segStartYIn);
        if (distThisSeg >= Math.abs(driveTargetIn)) {
          DRIVETRAIN.stop();
          headingHoldStop();
          ELEVATOR.ejectCoral();                // eject with scaled power (set in autonomousInit)
          autoTimer.reset(); autoTimer.start();
          autoState = AutoState.EJECT;
          System.out.println("[Auto L2] MOVE_FWD complete (" + distThisSeg + " in) → EJECT");
        }
        break;
      }

      case EJECT: {
        if (autoTimer.get() > EJECT_TIME_S) {
          ELEVATOR.setEjection(false);
          segStartYInBack = odomYIn;            // begin backoff segment after eject finishes
          headingHoldStart();                   // lock heading for the reverse, too
          autoTimer.reset(); autoTimer.start();
          autoState = AutoState.BACKOFF;
          System.out.println("[Auto L2] EJECT complete → BACKOFF");
        }
        break;
      }

      case BACKOFF: {
        // retreat opposite of forward direction (magnitude matches MoveCmd), to ~backoffIn
        double backCmd = -getMoveCmd();         // opposite direction
        currentFwdCmd = backCmd;
        double omega = headingHoldOmega();
        DRIVETRAIN.drive(0.0, currentFwdCmd, omega, true);

        double distBack = Math.abs(odomYIn - segStartYInBack);
        if (distBack >= Math.abs(backoffIn)) {
          DRIVETRAIN.stop();
          headingHoldStop();
          autoState = AutoState.DONE;
          System.out.println("[Auto L2] BACKOFF complete (" + distBack + " in) → DONE");
        }
        break;
      }

      case WAIT_FOR_HEIGHT: // not used in L2 path
      case DONE: {
        DRIVETRAIN.stop();
        break;
      }
    }
  }

  // ===== L3: move (raise in parallel) → wait-for-height → eject (heading hold on straight) =====
  private void runScore_Lx(double targetHeightIn, double driveTargetIn, double tolIn, double raiseTimeoutS) {
    switch (autoState) {
      case INIT: {
        DRIVETRAIN.stop();
        autoTimer.reset(); autoTimer.start();
        raiseDelayTimer.reset(); raiseDelayTimer.start();
        raiseIssued = false;
        segStartYIn = odomYIn;
        headingHoldStart(); // lock before we begin moving
        autoState = AutoState.MOVE_FWD;
        System.out.println("[Auto Lx] INIT → MOVE_FWD (targetDist=" + driveTargetIn + " in, height=" + targetHeightIn + " in)");
        break;
      }

      case MOVE_FWD: {
        // start raising after small delay so we clear barge before moving linkage
        if (!raiseIssued && raiseDelayTimer.get() >= getRaiseDelayS()) {
          ELEVATOR.setHeight(targetHeightIn);
          raiseIssued = true;
          System.out.println("[Auto Lx] RAISE command issued");
        }

        // drive forward at MoveCmd (field relative) with heading hold
        currentFwdCmd = getMoveCmd();
        double omega = headingHoldOmega();
        DRIVETRAIN.drive(0.0, currentFwdCmd, omega, true);

        // distance reached?
        double distThisSeg = Math.abs(odomYIn - segStartYIn);
        if (distThisSeg >= Math.abs(driveTargetIn)) {
          DRIVETRAIN.stop();
          headingHoldStop();
          autoTimer.reset(); autoTimer.start();
          autoState = AutoState.WAIT_FOR_HEIGHT;
          System.out.println("[Auto Lx] MOVE_FWD complete (" + distThisSeg + " in) → WAIT_FOR_HEIGHT");
        }
        break;
      }

      case WAIT_FOR_HEIGHT: {
        boolean atHeight = ELEVATOR.atHeightInches(targetHeightIn, tolIn);
        boolean timeout  = autoTimer.get() > raiseTimeoutS;
        if (atHeight || timeout) {
          if (timeout) System.out.println("[Auto Lx] WAIT_FOR_HEIGHT timeout—continuing");
          ELEVATOR.ejectCoral();
          autoTimer.reset(); autoTimer.start();
          autoState = AutoState.EJECT;
          System.out.println("[Auto Lx] WAIT_FOR_HEIGHT → EJECT");
        }
        break;
      }

      case EJECT: {
        if (autoTimer.get() > EJECT_TIME_S) {
          ELEVATOR.setEjection(false);
          DRIVETRAIN.stop();
          autoState = AutoState.DONE;
          System.out.println("[Auto Lx] EJECT → DONE");
        }
        break;
      }

      case BACKOFF: // not used here
      case DONE: {
        DRIVETRAIN.stop();
        break;
      }
    }
  }

  // ===== L4 (SAFE): raise to L2 → drive forward → back off → raise to L4 → eject =====
  private void runScore_L4_Safe() {
    final double l2HeightIn   = Constants.ELEVATOR_HEIGHTS[2];
    final double l4HeightIn   = Constants.ELEVATOR_HEIGHTS[4];
    final double driveTargetIn = getTargetL4In();
    final double backoffIn     = getBackoffL4In();

    switch (autoState) {
      case INIT: {
        DRIVETRAIN.stop();
        // 1) Raise to L2 FIRST (so we clear while approaching)
        ELEVATOR.setHeight(l2HeightIn);
        autoTimer.reset(); autoTimer.start();
        autoState = AutoState.RAISE_FIRST;
        System.out.println("[Auto L4 SAFE] INIT → RAISE_FIRST (to L2=" + l2HeightIn + " in)");
        break;
      }

      case RAISE_FIRST: { // waiting for L2
        boolean atL2   = ELEVATOR.atHeightInches(l2HeightIn, L2_TOL_IN);
        boolean timeout = autoTimer.get() > L2_RAISE_TIMEOUT_S;
        if (atL2 || timeout) {
          if (timeout) System.out.println("[Auto L4 SAFE] RAISE_FIRST timeout—continuing");
          segStartYIn = odomYIn;
          headingHoldStart();                // keep straight on the approach
          autoTimer.reset(); autoTimer.start();
          autoState = AutoState.MOVE_FWD;
          System.out.println("[Auto L4 SAFE] RAISE_FIRST → MOVE_FWD (targetDist=" + driveTargetIn + " in)");
        }
        break;
      }

      case MOVE_FWD: { // 2) Drive to reef at L2
        currentFwdCmd = getMoveCmd();
        double omega = headingHoldOmega();
        DRIVETRAIN.drive(0.0, currentFwdCmd, omega, true);

        double distThisSeg = Math.abs(odomYIn - segStartYIn);
        if (distThisSeg >= Math.abs(driveTargetIn)) {
          DRIVETRAIN.stop();
          headingHoldStop();
          segStartYInBack = odomYIn;
          // 3) Back off before raising to L4 (avoid barge)
          headingHoldStart();               // hold heading for the reverse
          autoTimer.reset(); autoTimer.start();
          autoState = AutoState.BACKOFF;
          System.out.println("[Auto L4 SAFE] MOVE_FWD complete (" + distThisSeg + " in) → BACKOFF");
        }
        break;
      }

      case BACKOFF: { // 3) Retreat a bit
        double backCmd = -getMoveCmd();
        currentFwdCmd = backCmd;
        double omega = headingHoldOmega();
        DRIVETRAIN.drive(0.0, currentFwdCmd, omega, true);

        double distBack = Math.abs(odomYIn - segStartYInBack);
        if (distBack >= Math.abs(backoffIn)) {
          DRIVETRAIN.stop();
          headingHoldStop();
          // 4) Now raise to L4 safely away from the reef hardware
          ELEVATOR.setHeight(l4HeightIn);
          autoTimer.reset(); autoTimer.start();
          autoState = AutoState.WAIT_FOR_HEIGHT;  // (wait for L4)
          System.out.println("[Auto L4 SAFE] BACKOFF complete (" + distBack + " in) → WAIT_FOR_HEIGHT (to L4)");
        }
        break;
      }

      case WAIT_FOR_HEIGHT: { // 4) Waiting for L4
        boolean atL4   = ELEVATOR.atHeightInches(l4HeightIn, L4_TOL_IN);
        boolean timeout = autoTimer.get() > L4_RAISE_TIMEOUT_S;
        if (atL4 || timeout) {
          if (timeout) System.out.println("[Auto L4 SAFE] WAIT_FOR_HEIGHT(L4) timeout—continuing");
          // 5) Shoot
          ELEVATOR.ejectCoral();
          autoTimer.reset(); autoTimer.start();
          autoState = AutoState.EJECT;
          System.out.println("[Auto L4 SAFE] WAIT_FOR_HEIGHT → EJECT");
        }
        break;
      }

      case EJECT: {
        if (autoTimer.get() > EJECT_TIME_S) {
          ELEVATOR.setEjection(false);
          DRIVETRAIN.stop();
          autoState = AutoState.DONE;
          System.out.println("[Auto L4 SAFE] EJECT → DONE");
        }
        break;
      }

      case DONE: {
        DRIVETRAIN.stop();
        break;
      }
    }
  }

  @Override
  public void teleopInit() {
    if (!orientedForMatch) {
      DRIVETRAIN.orientFacingDriverStation();
      orientedForMatch = true;
    }

    // Reset any auto-specific shot scaling so teleop shots are normal
    SmartDashboard.putNumber("Elevator/ShotPowerScale", 1.0);

    // seed climb tunables
    SmartDashboard.putNumber("Climb/ZeroOffsetDeg",     DEFAULT_ZERO_OFFSET_DEG);
    SmartDashboard.putNumber("Climb/ParkAfterZeroDeg",  DEFAULT_PARK_AFTER_ZERO_DEG);
    SmartDashboard.putNumber("Climb/HomeStallA",        DEFAULT_HOME_STALL_A);
    SmartDashboard.putNumber("Climb/HomeDebounceS",     DEFAULT_HOME_DEBOUNCE_S);
    SmartDashboard.putNumber("Climb/HomeBackoffDeg",    DEFAULT_HOME_BACKOFF_DEG);
    SmartDashboard.putNumber("Climb/HomeSpeedMag",      DEFAULT_HOME_SPEED_MAG);

    // >>> seed jog tunables (base + per-direction multipliers)
    SmartDashboard.putNumber("Climb/ManualJogPercent",  DEFAULT_MANUAL_JOG_PERCENT);
    SmartDashboard.putNumber("Climb/ManualJogOutMult",  DEFAULT_MANUAL_JOG_OUT_MULT);
    SmartDashboard.putNumber("Climb/ManualJogInMult",   DEFAULT_MANUAL_JOG_IN_MULT);

    SmartDashboard.putNumber("Climb/HTogglePercent",    DEFAULT_H_TOGGLE_PERCENT);
  }

  @Override
  public void teleopPeriodic() {
    // ===== Presets with algae offsets =====
    if (CONTROLLER.getCrossButtonPressed()) {
      ELEVATOR.setHeight(Constants.ELEVATOR_HEIGHTS[1]); // L1 unchanged
    } else if (CONTROLLER.getSquareButtonPressed()) {
      double h = Constants.ELEVATOR_HEIGHTS[2];          // L2
      if (ELEVATOR.inAlgaeMode()) h += 2.0;              // +2" in algae mode
      ELEVATOR.setHeight(h);
    } else if (CONTROLLER.getCircleButtonPressed()) {
      double h = Constants.ELEVATOR_HEIGHTS[3];          // L3
      if (ELEVATOR.inAlgaeMode()) h += 1.0;              // +1" in algae mode
      ELEVATOR.setHeight(h);
    } else if (CONTROLLER.getTriangleButtonPressed()) {
      ELEVATOR.setHeight(Constants.ELEVATOR_HEIGHTS[4]); // L4 unchanged
    } else if (CONTROLLER.getPOV() == 90) {
      ELEVATOR.setHeight(Constants.ELEVATOR_HEIGHTS[0]); // L0 unchanged
    }

    if (CONTROLLER.getR2ButtonPressed()) {
      if (!ELEVATOR.inAlgaeMode()) ELEVATOR.ejectCoral();
      else ELEVATOR.setEjection(!ELEVATOR.isEjecting());
    }
    if (CONTROLLER.getPSButtonPressed()) autoaim = !autoaim;
    if (CONTROLLER.getL2ButtonPressed()) ELEVATOR.setAlgaeMode(!ELEVATOR.inAlgaeMode());
    if (CONTROLLER.getTouchpadButtonPressed()) DRIVETRAIN.zeroGyro();

    // ===== Drive shaping =====
    double rawX   = CONTROLLER.getLeftX();
    double rawY   = CONTROLLER.getLeftY();
    double rawRot = CONTROLLER.getRightX();

    double xCmd   = shapeInput(rawX,   TRANS_DEADBAND, TRANS_EXPO);
    double yCmd   = shapeInput(rawY,   TRANS_DEADBAND, TRANS_EXPO);
    double rotCmd = shapeInput(rawRot, ROT_DEADBAND,   ROT_EXPO);

    xCmd   = xLimiter.calculate(xCmd);
    yCmd   = yLimiter.calculate(yCmd);
    rotCmd = rotLimiter.calculate(rotCmd) * ROT_GAIN;

    double finalOmega = autoaim ? LimelightHelpers.getTX("") : rotCmd;
    DRIVETRAIN.drive(xCmd, yCmd, finalOmega, foc);

    // ===== POV handling & climb state =====
    int pov = CONTROLLER.getPOV();
    boolean downHeld  = (pov == 180); // INWARD
    boolean upHeld    = (pov ==   0); // OUTWARD
    boolean leftEdge  = (pov == 270) && (lastPOV != 270);

    boolean r3Pressed = CONTROLLER.getR3ButtonPressed();

    SmartDashboard.putBoolean("Climb/DpadDownHeld",   downHeld);
    SmartDashboard.putBoolean("Climb/DpadUpHeld",     upHeld);
    SmartDashboard.putBoolean("Climb/DpadLeftPressed", leftEdge);
    SmartDashboard.putBoolean("Climb/HToggleActive",  hToggleActive);

    if (leftEdge) {
      if (phase == Phase.IDLE) {
        System.out.println("[Climb] LEFT: start auto-climb (auto-direction probe)");
        CLIMBER.stopAll();
        CLIMBER.runHPercent(ClimberSubsystem.INTAKE_SPEED);
        CLIMBER.disableSoftLimits();

        homeDirSign = +1.0;
        homeDirLearned = false;
        probeStartDeg = CLIMBER.getGBPositionDegrees();
        CLIMBER.runGBPercent(homeDirSign * Math.abs(getHomeSpeedMag()));

        timer.restart();
        debounceStart = -1.0;
        phase = Phase.PROBE;
      } else {
        System.out.println("[Climb] LEFT: cancel auto-climb");
        CLIMBER.stopAll();
        phase = Phase.IDLE;
      }
    }

    // >>> Faster manual jog — separate OUT/IN multipliers (clamped ≤ 1.0)
    if (phase == Phase.IDLE) {
      double base = Math.abs(getManualJogPercent());     // 0..1
      double outM = Math.abs(getManualJogOutMult());     // ≥1
      double inM  = Math.abs(getManualJogInMult());      // ≥1
      double jogOut = Math.min(1.0, base * outM);
      double jogIn  = Math.min(1.0, base * inM);

      if (upHeld ^ downHeld) {
        CLIMBER.stopH(); // reduce draw during manual tweak
        if (upHeld) {
          // OUTWARD = opposite of homing sign
          CLIMBER.runGBPercent(-homeDirSign * jogOut);
        } else {
          // INWARD  = same as homing sign
          CLIMBER.runGBPercent(+homeDirSign * jogIn);
        }
      } else {
        CLIMBER.stopGB();
      }
    }

    // H toggle (unchanged)
    if (phase == Phase.IDLE && r3Pressed) {
      hToggleActive = !hToggleActive;
      if (hToggleActive) CLIMBER.runHPercent(getHTogglePercent());
      else CLIMBER.stopH();
    }
    if (phase == Phase.IDLE && hToggleActive && !upHeld && !downHeld) {
      CLIMBER.runHPercent(getHTogglePercent());
    }

    // Auto-climb state machine (unchanged)
    switch (phase) {
      case IDLE: break;

      case PROBE: {
        double ampsStator = CLIMBER.getGBStatorCurrent();
        boolean over = ampsStator >= getHomeStallA();
        if (over) {
          if (debounceStart < 0) debounceStart = timer.get();
          if (timer.get() - debounceStart >= Math.min(0.10, getHomeDebounceS())) {
            System.out.println("[Climb] PROBE: stall detected (direction OK)");
            homeDirLearned = true;
            phase = Phase.HOME_PUSH;
            timer.restart();
            break;
          }
        } else {
          debounceStart = -1.0;
        }

        if (timer.get() >= PROBE_TIME_S && !homeDirLearned) {
          double moved = Math.abs(CLIMBER.getGBPositionDegrees() - probeStartDeg);
          if (moved >= PROBE_MIN_TRAVEL_DEG) {
            homeDirSign *= -1.0;
            System.out.println("[Climb] PROBE: flipping homing direction");
            CLIMBER.runGBPercent(homeDirSign * Math.abs(getHomeSpeedMag()));
            timer.restart();
            probeStartDeg = CLIMBER.getGBPositionDegrees();
            phase = Phase.HOME_PUSH;
          } else {
            phase = Phase.HOME_PUSH;
          }
        }
        break;
      }

      case HOME_PUSH: {
        double ampsStator = CLIMBER.getGBStatorCurrent();
        boolean over = ampsStator >= getHomeStallA();
        if (over) {
          if (debounceStart < 0) debounceStart = timer.get();
          if (timer.get() - debounceStart >= getHomeDebounceS()) {
            System.out.println("[Climb] HOME: stall -> stop + backoff");
            CLIMBER.stopGB();
            double backoffTgt = CLIMBER.getGBPositionDegrees() - homeDirSign * getHomeBackoffDeg();
            CLIMBER.commandGBToDegrees(backoffTgt);
            timer.restart();
            phase = Phase.HOME_BACKOFF;
          }
        } else {
          debounceStart = -1.0;
        }
        if (timer.get() > 4.0) {
          System.out.println("[Climb] HOME: timeout — stopping");
          CLIMBER.stopGB();
          phase = Phase.IDLE;
        }
        break;
      }

      case HOME_BACKOFF: {
        if (timer.get() >= (getHomeDebounceS() + 0.10)) {
          boolean ok = CLIMBER.calibrateGBPositionDegrees(getZeroOffsetDeg());
          System.out.println("[Climb] Zero write " + (ok ? "OK" : "FAILED"));
          CLIMBER.commandGBToDegrees(getParkAfterZeroDeg());
          timer.restart();
          phase = Phase.CALIBRATE_AND_PARK;
        }
        break;
      }

      case CALIBRATE_AND_PARK: {
        if (CLIMBER.isGBAtDegrees(getParkAfterZeroDeg())) {
          System.out.println("[Climb] Park reached — homing complete");
          phase = Phase.IDLE;
          lastPOV = -1;
        }
        if (timer.get() > 3.0) {
          System.out.println("[Climb] Park timeout — returning to IDLE");
          phase = Phase.IDLE;
        }
        break;
      }
    }

    SmartDashboard.putNumber("PS5 POV", pov);
    SmartDashboard.putString("Climb/Phase", phase.name());
    SmartDashboard.putNumber("GB Pos Deg", CLIMBER.getGBPositionDegrees());
    SmartDashboard.putNumber("GB Stator A", CLIMBER.getGBStatorCurrent());
    SmartDashboard.putNumber("GB Supply A", CLIMBER.getGBSupplyCurrent());
    SmartDashboard.putNumber("H Supply A",  CLIMBER.getHSupplyCurrent());

    lastPOV = pov;
  }

  @Override
  public void disabledInit() {
    if (CLIMBER != null) CLIMBER.stopAll();
    hToggleActive = false;
  }
  @Override public void disabledPeriodic() {}
  @Override public void testInit() {}
  @Override public void testPeriodic() {}

  // ==== helpers ====
  private static double shapeInput(double raw, double deadband, double expo) {
    double v = MathUtil.applyDeadband(raw, deadband);
    return Math.copySign(Math.pow(Math.abs(v), expo), v);
  }
}
