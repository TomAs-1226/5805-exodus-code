// FULL FILE — Robot.java
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
import frc.robot.vision.AutoAlignLL;
import edu.wpi.first.networktables.GenericEntry;



import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import java.util.Map;

import com.ctre.phoenix6.hardware.Pigeon2;


public class Robot extends TimedRobot {
  private boolean doStartupWheelZero = false;
  private double  startupWheelZeroUntilSec = 0.0;

  private final CommandSystem COMSYS;
  private final Elevator ELEVATOR;
  private final SwerveDrivetrain DRIVETRAIN;
  private final ClimberSubsystem CLIMBER;

  private static final PS5Controller CONTROLLER = new PS5Controller(0);

  // CTRE field-centric (no manual matrix)
  private static final double ROT_GAIN   = 0.80;
  private enum L4Phase {
    RAISE_L2,
    HOLD_L2,
    DRIVE_FWD,
    HOLD_AT_REEF,
    RAISE_L4,
    SHOOT,
    BACKOFF,
    DONE
}
private L4Phase l4Phase = L4Phase.RAISE_L2;

// segment distance trackers for L4 auto
private double l4SegStartYIn = 0.0;
private double l4SegStartYInBack = 0.0;

// flags to handle "wait .5s after reaching height"
private boolean l4L2Reached = false;
private boolean l4L4Reached = false;
// driver UI widgets
private GenericEntry algaeModeBoxEntry;
private GenericEntry climberHBoxEntry;


  // Limelight
  private static final String LL_NAME = "limelight"; 
  private static final int    LL_PIPELINE_INDEX = 0;
  private final AutoAlignLL ALIGN = new AutoAlignLL(LL_NAME, LL_PIPELINE_INDEX);

  // Driver shaping
  private static final double TRANS_DEADBAND = 0.08;
  private static final double ROT_DEADBAND   = 0.08;
  private static final double TRANS_EXPO     = 2.0;
  private static final double ROT_EXPO       = 2.4;
  private static final double TRANS_SLEW     = 3.0;
  private static final double ROT_SLEW       = 2.0;
  private static final double IDLE_BAND      = 0.02;

  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(TRANS_SLEW);
  private final SlewRateLimiter fwdLimiter    = new SlewRateLimiter(TRANS_SLEW);
  private final SlewRateLimiter rotLimiter    = new SlewRateLimiter(ROT_SLEW);

  // Auto chooser
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private String autoSelected = "do_nothing";

  // Auto distances (unchanged)
  private static final double DEFAULT_IPS_PER_CMD          = 120.0;
  private static final double DEFAULT_MOVE_CMD             = 0.35;
  private static final double DEFAULT_RAISE_DELAY_S        = 0.25;

  private static final double DEFAULT_TARGET_DIST_L2_IN    = 41.5;
  private static final double DEFAULT_BACKOFF_L2_IN        = 7.0;

  private static final double DEFAULT_TARGET_DIST_L3_IN    = 80.0;

  private static final double DEFAULT_SHOT_SCALE_L2        = 0.85;
  private static final double DEFAULT_LEAVE_DIST_IN        = 44.0;
  // simple straight-drive autos
private static final double DEFAULT_SIMPLE_FWD_DIST_IN         = 24.0; // how far "fwd6" should actually go
private static final double DEFAULT_SIMPLE_FWD_AFTERL2_DIST_IN = 24.0; // how far "l2_then_fwd6" drives after raising L2

// buddy auto distances
private static final double DEFAULT_BUDDY_FWD_IN               = 24.0; // forward ~1 ft
private static final double DEFAULT_BUDDY_BACK_IN              = 48.0; // back ~3 ft

  private static final double EJECT_TIME_S                 = 0.60;
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

  private double getTargetL3In()     { return SmartDashboard.getNumber("Auto/TargetDistL3In", DEFAULT_TARGET_DIST_L3_IN); }

  private double getLeaveDistIn()    { return SmartDashboard.getNumber("Auto/LeaveDistIn",   DEFAULT_LEAVE_DIST_IN); }
  private double getSimpleFwdDistIn() {
    return SmartDashboard.getNumber("Auto/SimpleFwdDistIn", DEFAULT_SIMPLE_FWD_DIST_IN);
  }
  private double getSimpleFwdAfterL2DistIn() {
    return SmartDashboard.getNumber("Auto/SimpleFwdAfterL2DistIn", DEFAULT_SIMPLE_FWD_AFTERL2_DIST_IN);
  }
  private double getBuddyFwdDistIn() {
    return SmartDashboard.getNumber("Auto/BuddyFwdIn", DEFAULT_BUDDY_FWD_IN);
  }
  private double getBuddyBackDistIn() {
    return SmartDashboard.getNumber("Auto/BuddyBackIn", DEFAULT_BUDDY_BACK_IN);
  }
  

  // Heading-hold (autos) – simple PID on IMU yaw
  private static final double HEAD_KP = 0.02;
  private static final double HEAD_KI = 0.00;
  private static final double HEAD_KD = 0.001;

  private final PIDController headingPid = new PIDController(HEAD_KP, HEAD_KI, HEAD_KD);
  private double headingTargetDeg = 0.0;
  private boolean headingLocked = false;

  // Pigeon2 (for autos PID only; CTRE drivetrain handles field-centric internally)
  private static final String PIGEON_CANBUS = "*";
  private final Pigeon2 IMU = new Pigeon2(Constants.Device.PIGEON_2.ID, PIGEON_CANBUS);

  private void headingHoldStart() {
    headingTargetDeg = IMU.getRotation2d().getDegrees(); // CCW+, NWU
    headingPid.reset();
    headingPid.enableContinuousInput(-180.0, 180.0);
    headingLocked = true;
  }
  private void headingHoldStop() { headingLocked = false; }
  private double headingHoldOmega() {
    if (!headingLocked) return 0.0;
    double yawDeg = IMU.getRotation2d().getDegrees();
    double cmd = headingPid.calculate(yawDeg, headingTargetDeg);
    return MathUtil.clamp(cmd, -1.0, 1.0);
  }

  private enum AutoState { INIT, RAISE_FIRST, MOVE_FWD, WAIT_FOR_HEIGHT, BACKOFF, EJECT, DONE }
  private AutoState autoState = AutoState.INIT;
  

  private final Timer autoTimer = new Timer();
  private final Timer raiseDelayTimer = new Timer();


  private double odomYIn = 0.0;
  private double segStartYIn = 0.0;
  private double segStartYInBack = 0.0;
  private double lastTSec = 0.0;
  private double currentFwdCmd = 0.0;
  private boolean raiseIssued = false;

  private enum Phase { IDLE }
  private Phase phase = Phase.IDLE;
  private final Timer timer = new Timer();
  private int lastPOV = -1;
  private double homeDirSign = +1.0;


  


  public Robot() {
    this.COMSYS     = new CommandSystem(this);
    this.ELEVATOR   = SubsystemManager.registerSubsystem(Elevator::new);
    this.DRIVETRAIN = SubsystemManager.registerSubsystem(SwerveDrivetrain::new);
    this.CLIMBER    = new ClimberSubsystem();
    SubsystemManager.start();
  }

  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Drive/CoR_X_in", 0.0);
    SmartDashboard.putNumber("Drive/CoR_Y_in", 0.0);
    SmartDashboard.putNumber("Drive/SideTrim", 0.0);
    SmartDashboard.putBoolean("Drive/ReverseFieldForward", true); // start reversed if you want field-backwards feel

    // Optional hand-tuned forward nudge (e.g., +5 if physical forward is slightly off)
    SmartDashboard.putNumber("Drive/HeadingOffsetDeg", 0.0);

    // Quick calibration helpers if your IMU mounting gave you a 90° slip
    SmartDashboard.putNumber("Drive/YawCalibDeg", 0.0);
    SmartDashboard.putBoolean("Drive/Apply90Fix", false);

    // Only one rotate toggle now (stick sense). Output sign is fixed.
    SmartDashboard.putBoolean("Drive/InvertRotStick",  false);

    // Auto chooser
    autoChooser.setDefaultOption("Do Nothing", "do_nothing");
    //autoChooser.addOption("Leave", "leave");
    autoChooser.addOption("Score L2 (backoff 6\")", "score_l2_backoff6");
    //autoChooser.addOption("Score L3", "score_l3");
    autoChooser.addOption("Score L4 (safe L2-first)", "score_l4_safe");
    autoChooser.addOption("Forward 6 in", "fwd6");
    autoChooser.addOption("L2 then Forward 6 in", "l2_then_fwd6");
    autoChooser.addOption("Buddy Auto", "buddy_auto");

    SmartDashboard.putData("Auto Selector", autoChooser);
    Shuffleboard.getTab("Autonomous")
      .add("Auto Selector", autoChooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser);

    // Seed auto tunables
    //SmartDashboard.putNumber("Auto/IPSPerCmd",   DEFAULT_IPS_PER_CMD);
    //SmartDashboard.putNumber("Auto/MoveCmd",     DEFAULT_MOVE_CMD);
    //SmartDashboard.putNumber("Auto/RaiseDelayS", DEFAULT_RAISE_DELAY_S);

    SmartDashboard.putNumber("Auto/TargetDistL2In", DEFAULT_TARGET_DIST_L2_IN);
    SmartDashboard.putNumber("Auto/BackoffL2In",    DEFAULT_BACKOFF_L2_IN);
    //SmartDashboard.putNumber("Auto/ShotPowerScaleL2", DEFAULT_SHOT_SCALE_L2);

    //SmartDashboard.putNumber("Auto/TargetDistL3In", DEFAULT_TARGET_DIST_L3_IN);
    //SmartDashboard.putNumber("Auto/TargetDistL4In", DEFAULT_TARGET_DIST_L4_IN);
    //SmartDashboard.putNumber("Auto/BackoffL4In",    DEFAULT_BACKOFF_L4_IN);

    SmartDashboard.putNumber("Auto/LeaveDistIn",    DEFAULT_LEAVE_DIST_IN);
    SmartDashboard.putNumber("Auto/SimpleFwdDistIn",         DEFAULT_SIMPLE_FWD_DIST_IN);
SmartDashboard.putNumber("Auto/SimpleFwdAfterL2DistIn",  DEFAULT_SIMPLE_FWD_AFTERL2_DIST_IN);
SmartDashboard.putNumber("Auto/BuddyFwdIn",              DEFAULT_BUDDY_FWD_IN);
SmartDashboard.putNumber("Auto/BuddyBackIn",             DEFAULT_BUDDY_BACK_IN);
    SmartDashboard.putNumber("Auto/OdomYIn", 0.0);
        // === DriverUI tab widgets for mode/climber state ===
    // This forces Shuffleboard to create and keep these entries every boot.
    // === DriverUI tab widgets (big color boxes) ===
    // We use kBooleanBox:
    //   true  -> GREEN
    //   false -> RED
    var driverTab = Shuffleboard.getTab("DriverUI");

    algaeModeBoxEntry = driverTab
        .add("ALGAE MODE", false) // false = coral mode (red box at start)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of(
            "Color when true", "Lime",
            "Color when false", "Red"
        ))
        .withPosition(0, 0) // big and obvious for drive coach
        .withSize(4, 3)     // make it LARGE
        .getEntry();

    climberHBoxEntry = driverTab
        .add("CLIMBER H", false) // false = not spinning yet
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of(
            "Color when true", "Lime",
            "Color when false", "Red"
        ))
        .withPosition(4, 0) // right next to algae box
        .withSize(4, 3)
        .getEntry();


  }

  @Override public void robotPeriodic() { SubsystemManager.update(); }

  @Override
  public void autonomousInit() {
    DRIVETRAIN.start();
    DRIVETRAIN.setDriveMaxAll(0.90);
    DRIVETRAIN.setSteerMaxAll(0.95);
// --- Field-centric forward = away from our driver station (toward barge) ---
final double headingOffsetDeg = SmartDashboard.getNumber("Drive/HeadingOffsetDeg", 0.0);
double yawCalib = SmartDashboard.getNumber("Drive/YawCalibDeg", 0.0);
if (SmartDashboard.getBoolean("Drive/Apply90Fix", false)) {
    yawCalib += 90.0;
}

// tell drivetrain what "forward" is for this alliance (away from DS)
DRIVETRAIN.setDriverForwardOffsetDegrees(headingOffsetDeg + yawCalib);
DRIVETRAIN.setOperatorPerspectiveForAlliance();
DRIVETRAIN.seedFieldCentricNow();

    autoSelected = autoChooser.getSelected();
    odomYIn = 0.0;
    segStartYIn = 0.0;
    segStartYInBack = 0.0;
    currentFwdCmd = 0.0;
    lastTSec = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Auto/OdomYIn", odomYIn);

    autoTimer.stop(); autoTimer.reset();
    raiseIssued = false;
    raiseDelayTimer.stop(); raiseDelayTimer.reset();

    SmartDashboard.putNumber("Elevator/ShotPowerScale",
        "score_l2_backoff6".equals(autoSelected) ? DEFAULT_SHOT_SCALE_L2 : 1.0);

    headingHoldStop();
    autoState = AutoState.INIT;
    // reset L4 auto machine
l4Phase = L4Phase.RAISE_L2;
l4SegStartYIn = 0.0;
l4SegStartYInBack = 0.0;
l4L2Reached = false;
l4L4Reached = false;

  }

  @Override
  public void autonomousPeriodic() {
    double now = Timer.getFPGATimestamp();
    double dt  = now - lastTSec;
    lastTSec   = now;
    odomYIn   += currentFwdCmd * getIPSPerCmd() * dt;
    SmartDashboard.putNumber("Auto/OdomYIn", odomYIn);
    currentFwdCmd = 0.0;

    switch (autoSelected) {
      case "leave":            runLeaveForward(); break;
      case "score_l2_backoff6":runScoreL2_RaiseThenGo_Eject_Backoff6(); break;
      case "score_l3":         runScore_Lx(Constants.ELEVATOR_HEIGHTS[3], getTargetL3In(), L3_TOL_IN, L3_RAISE_TIMEOUT_S); break;
      case "score_l4_safe":    runScore_L4_Safe(); break;
      case "fwd6":            runForward6In(); break;
      case "l2_then_fwd6":    runRaiseL2_ThenForward6In(); break;
      case "buddy_auto":      runBuddyAuto();    break;

      default:                 DRIVETRAIN.drive(0,0,0,true); break;
    }
  }

  // ==== Buddy Auto: forward ~1ft, then back ~3ft ====
private void runBuddyAuto() {
  final double forwardTargetIn = getBuddyFwdDistIn();   // default 12 in
  final double backTargetIn    = getBuddyBackDistIn();  // default 36 in
  final double maxFwdTimeS     = 4.0;
  final double maxBackTimeS    = 6.0;

  switch (autoState) {
    case INIT:
      DRIVETRAIN.drive(0,0,0,true);
      segStartYIn = odomYIn;
      headingHoldStart();
      autoTimer.reset(); autoTimer.start();
      autoState = AutoState.MOVE_FWD;
      break;

    case MOVE_FWD:
      // drive forward
      currentFwdCmd = -getMoveCmd();
      DRIVETRAIN.drive(currentFwdCmd, 0.0, headingHoldOmega(), true);

      boolean fwdReached  = Math.abs(odomYIn - segStartYIn) >= Math.abs(forwardTargetIn);
      boolean fwdTimedOut = autoTimer.get() > maxFwdTimeS;
      if (fwdReached || fwdTimedOut) {
        // stop forward, prep for reverse
        DRIVETRAIN.drive(0,0,0,true);
        headingHoldStop();

        segStartYInBack = odomYIn;
        headingHoldStart();
        autoTimer.reset(); autoTimer.start();
        autoState = AutoState.BACKOFF;
      }
      break;

    case BACKOFF:
      // drive backward
      currentFwdCmd = getMoveCmd();
      DRIVETRAIN.drive(currentFwdCmd, 0.0, headingHoldOmega(), true);

      boolean backReached  = Math.abs(odomYIn - segStartYInBack) >= Math.abs(backTargetIn);
      boolean backTimedOut = autoTimer.get() > maxBackTimeS;
      if (backReached || backTimedOut) {
        DRIVETRAIN.drive(0,0,0,true);
        headingHoldStop();
        autoState = AutoState.DONE;
      }
      break;

    default:
      DRIVETRAIN.drive(0,0,0,true);
      break;
  }
}

  // ==== Simple forward 6 inches (auto) ====
private void runForward6In() {
  final double driveTargetIn   = getSimpleFwdDistIn();       // fixed 6 inches
  final double maxSegmentTimeS = 4;        // simple safety timeout

  switch (autoState) {
    case INIT:
      DRIVETRAIN.drive(0,0,0,true);
      segStartYIn = odomYIn;
      headingHoldStart();
      autoTimer.reset(); autoTimer.start();
      autoState = AutoState.MOVE_FWD;
      break;

    case MOVE_FWD:
      currentFwdCmd = -getMoveCmd(); // reuse your dashboard-tunable forward command
      DRIVETRAIN.drive(currentFwdCmd, 0.0, headingHoldOmega(), true);
      boolean reached = Math.abs(odomYIn - segStartYIn) >= Math.abs(driveTargetIn);
      boolean timedOut = autoTimer.get() > maxSegmentTimeS;
      if (reached || timedOut) {
        DRIVETRAIN.drive(0,0,0,true);
        headingHoldStop();
        autoState = AutoState.DONE;
      }
      break;
      
    default:
      DRIVETRAIN.drive(0,0,0,true);
      break;
  }
}

// ==== Raise to L2, then forward 6 inches (auto) ====
private void runRaiseL2_ThenForward6In() {
  final double l2HeightIn      = Constants.ELEVATOR_HEIGHTS[2];
  final double driveTargetIn   = getSimpleFwdAfterL2DistIn();
  final double raiseTimeoutS   = L2_RAISE_TIMEOUT_S;  // you already define this
  final double tolIn           = L2_TOL_IN;           // you already define this
  final double maxSegmentTimeS = 20;                 // safety for the drive segment

  switch (autoState) {
    case INIT:
      DRIVETRAIN.drive(0,0,0,true);
      ELEVATOR.setHeight(l2HeightIn);              // start raising
      autoTimer.reset(); autoTimer.start();
      autoState = AutoState.RAISE_FIRST;
      break;

    case RAISE_FIRST:
      // proceed once we're at L2 or we time out
      if (ELEVATOR.atHeightInches(l2HeightIn, tolIn) || autoTimer.get() > raiseTimeoutS) {
        segStartYIn = odomYIn;
        headingHoldStart();
        autoTimer.reset(); autoTimer.start();
        autoState = AutoState.MOVE_FWD;
      }
      break;

    case MOVE_FWD:
      currentFwdCmd = -getMoveCmd();
      DRIVETRAIN.drive(currentFwdCmd, 0.0, headingHoldOmega(), true);
      boolean reached = Math.abs(odomYIn - segStartYIn) >= Math.abs(driveTargetIn);
      boolean timedOut = autoTimer.get() > maxSegmentTimeS;
      if (reached || timedOut) {
        DRIVETRAIN.drive(0,0,0,true);
        headingHoldStop();
        autoState = AutoState.DONE;
      }
      break;

    default:
      DRIVETRAIN.drive(0,0,0,true);
      break;
  }
}

  // ==== Leave straight (auto) ====
  private void runLeaveForward() {
    final double driveTargetIn = getLeaveDistIn();
    switch (autoState) {
      case INIT:
        DRIVETRAIN.drive(0,0,0,true);
        segStartYIn = odomYIn;
        headingHoldStart();
        autoState = AutoState.MOVE_FWD;
        break;

      case MOVE_FWD:
        currentFwdCmd = getMoveCmd();
        DRIVETRAIN.drive(currentFwdCmd, 0.0, headingHoldOmega(), true);
        if (Math.abs(odomYIn - segStartYIn) >= Math.abs(driveTargetIn)) {
          DRIVETRAIN.drive(0,0,0,true);
          headingHoldStop();
          autoState = AutoState.DONE;
        }
        break;

      default:
        DRIVETRAIN.drive(0,0,0,true);
        break;
    }
  }

// ==== L2 (auto) with shoot delay ====
private void runScoreL2_RaiseThenGo_Eject_Backoff6() {
  final double targetHeightIn = Constants.ELEVATOR_HEIGHTS[2];
  final double driveTargetIn  = getTargetL2In();
  final double backoffIn      = getBackoffL2In();

  // how long to pause at the reef before actually spitting (seconds)
  final double SHOOT_DELAY_S  = 0.25;

  switch (autoState) {
    case INIT:
      DRIVETRAIN.drive(0,0,0,true);
      ELEVATOR.setHeight(targetHeightIn);
      autoTimer.reset(); autoTimer.start();
      autoState = AutoState.RAISE_FIRST;
      break;

    case RAISE_FIRST:
      if (ELEVATOR.atHeightInches(targetHeightIn, L2_TOL_IN) || autoTimer.get() > L2_RAISE_TIMEOUT_S) {
        segStartYIn = odomYIn;
        headingHoldStart();
        autoTimer.reset(); autoTimer.start();
        autoState = AutoState.MOVE_FWD;
      }
      break;

    case MOVE_FWD:
      currentFwdCmd = getMoveCmd();
      DRIVETRAIN.drive(currentFwdCmd, 0.0, headingHoldOmega(), true);

      if (Math.abs(odomYIn - segStartYIn) >= Math.abs(driveTargetIn)) {
        // we're in position, stop and hold heading
        DRIVETRAIN.drive(0,0,0,true);
        headingHoldStop();

        // start a short wait before we actually eject
        autoTimer.reset(); autoTimer.start();
        autoState = AutoState.WAIT_FOR_HEIGHT; // reuse this state as "pre-shoot delay"
      }
      break;

    case WAIT_FOR_HEIGHT:
      // just sit still for SHOOT_DELAY_S, THEN fire
      if (autoTimer.get() > SHOOT_DELAY_S) {
        ELEVATOR.ejectCoral();
        autoTimer.reset(); autoTimer.start();
        autoState = AutoState.EJECT;
      }
      break;

    case EJECT:
      if (autoTimer.get() > EJECT_TIME_S) {
        ELEVATOR.setEjection(false);
        segStartYInBack = odomYIn;
        headingHoldStart();
        autoTimer.reset(); autoTimer.start();
        autoState = AutoState.BACKOFF;
      }
      break;

    case BACKOFF:
      currentFwdCmd = -getMoveCmd();
      DRIVETRAIN.drive(currentFwdCmd, 0.0, headingHoldOmega(), true);

      if (Math.abs(odomYIn - segStartYInBack) >= Math.abs(backoffIn)) {
        DRIVETRAIN.drive(0,0,0,true);
        headingHoldStop();
        autoState = AutoState.DONE;
      }
      break;

    default:
      DRIVETRAIN.drive(0,0,0,true);
      break;
  }
}


  // ==== L3 generic (auto) ====
  private void runScore_Lx(double targetHeightIn, double driveTargetIn, double tolIn, double raiseTimeoutS) {
    switch (autoState) {
      case INIT:
        DRIVETRAIN.drive(0,0,0,true);
        autoTimer.reset(); autoTimer.start();
        raiseDelayTimer.reset(); raiseDelayTimer.start();
        raiseIssued = false;
        segStartYIn = odomYIn;
        headingHoldStart();
        autoState = AutoState.MOVE_FWD;
        break;

      case MOVE_FWD:
        if (!raiseIssued && raiseDelayTimer.get() >= getRaiseDelayS()) {
          ELEVATOR.setHeight(targetHeightIn);
          raiseIssued = true;
        }
        currentFwdCmd = getMoveCmd();
        DRIVETRAIN.drive(currentFwdCmd, 0.0, headingHoldOmega(), true);
        if (Math.abs(odomYIn - segStartYIn) >= Math.abs(driveTargetIn)) {
          DRIVETRAIN.drive(0,0,0,true);
          headingHoldStop();
          autoTimer.reset(); autoTimer.start();
          autoState = AutoState.WAIT_FOR_HEIGHT;
        }
        break;

      case WAIT_FOR_HEIGHT:
        if (ELEVATOR.atHeightInches(targetHeightIn, tolIn) || autoTimer.get() > raiseTimeoutS) {
          ELEVATOR.ejectCoral();
          autoTimer.reset(); autoTimer.start();
          autoState = AutoState.EJECT;
        }
        break;

      case EJECT:
        if (autoTimer.get() > EJECT_TIME_S) {
          ELEVATOR.setEjection(false);
          DRIVETRAIN.drive(0,0,0,true);
          autoState = AutoState.DONE;
        }
        break;

      default:
        DRIVETRAIN.drive(0,0,0,true);
        break;
    }
  }

// ==== L4 safe (auto) reworked ====
// sequence:
// 1. raise to L2
// 2. wait 0.5s
// 3. drive in to reef
// 4. wait 0.5s
// 5. raise to L4
// 6. short settle, then shoot
// 7. back off a tiny bit
// 8. after backing out, send elevator home (zero height)
private void runScore_L4_Safe() {
  final double l2HeightIn    = Constants.ELEVATOR_HEIGHTS[2];
  final double l4HeightIn    = Constants.ELEVATOR_HEIGHTS[4];

  // how far to drive IN and how far to back OUT
  final double driveTargetIn = getTargetL2In();
  final double backoffIn     = getBackoffL2In();

  // "wait a bit" before spitting after we're up at L4
  final double SHOOT_DELAY_S = 0.25;

  switch (l4Phase) {

    // command L2 height, start timing
    case RAISE_L2:
      DRIVETRAIN.drive(0,0,0,true);
      ELEVATOR.setHeight(l2HeightIn);

      l4L2Reached = false;              // haven't confirmed L2 yet
      autoTimer.reset(); autoTimer.start();
      l4Phase = L4Phase.HOLD_L2;
      break;

    // stay at L2, then wait 0.5s AFTER we actually reach it
    case HOLD_L2:
      currentFwdCmd = 0.0;

      boolean l2Ready = ELEVATOR.atHeightInches(l2HeightIn, L2_TOL_IN)
                      || autoTimer.get() > L2_RAISE_TIMEOUT_S;

      if (!l4L2Reached) {
          // first moment we consider L2 "good"
          if (l2Ready) {
              l4L2Reached = true;
              autoTimer.reset(); autoTimer.start(); // start the 0.5s dwell
          }
      } else {
          // we've already hit L2; now burn 0.5s before we drive forward
          if (autoTimer.get() >= 0.5) {
              l4SegStartYIn = odomYIn; // record where we started the forward push
              headingHoldStart();      // lock heading for the push
              autoTimer.reset(); autoTimer.start();
              l4Phase = L4Phase.DRIVE_FWD;
          }
      }
      break;

    // drive forward toward reef while holding heading
    case DRIVE_FWD:
      currentFwdCmd = getMoveCmd();
      DRIVETRAIN.drive(currentFwdCmd, 0.0, headingHoldOmega(), true);

      if (Math.abs(odomYIn - l4SegStartYIn) >= Math.abs(driveTargetIn)) {
          // reached reef, stop and pause
          DRIVETRAIN.drive(0,0,0,true);
          headingHoldStop();

          autoTimer.reset(); autoTimer.start(); // start the 0.5s dwell at reef
          l4Phase = L4Phase.HOLD_AT_REEF;
      }
      break;

    // hold still at reef for 0.5s, THEN go for L4 height
    case HOLD_AT_REEF:
      currentFwdCmd = 0.0;

      if (autoTimer.get() >= 0.5) {
          ELEVATOR.setHeight(l4HeightIn);

          l4L4Reached = false;          // haven't confirmed L4 yet
          autoTimer.reset(); autoTimer.start();
          l4Phase = L4Phase.RAISE_L4;
      }
      break;

    // raise to L4, then small SHOOT_DELAY_S settle, then spit
    case RAISE_L4:
      currentFwdCmd = 0.0;

      boolean l4Ready = ELEVATOR.atHeightInches(l4HeightIn, L4_TOL_IN)
                      || autoTimer.get() > L4_RAISE_TIMEOUT_S;

      if (!l4L4Reached) {
          // first time we believe we're basically at L4
          if (l4Ready) {
              l4L4Reached = true;
              autoTimer.reset(); autoTimer.start(); // start small pre-shoot delay
          }
      } else {
          // after reach L4, let it sit SHOOT_DELAY_S, then fire
          if (autoTimer.get() >= SHOOT_DELAY_S) {
              ELEVATOR.ejectCoral();    // start ejecting
              autoTimer.reset(); autoTimer.start();

              // prep for backing out
              l4SegStartYInBack = odomYIn;
              headingHoldStart();

              l4Phase = L4Phase.SHOOT;
          }
      }
      break;

    // keep shooting until EJECT_TIME_S, then stop and start backing up
    case SHOOT:
      currentFwdCmd = 0.0;

      if (autoTimer.get() > EJECT_TIME_S) {
          ELEVATOR.setEjection(false);

          autoTimer.reset(); autoTimer.start();
          l4Phase = L4Phase.BACKOFF;
      }
      break;

    // back off a tiny bit while holding heading
    case BACKOFF:
      currentFwdCmd = -getMoveCmd();
      DRIVETRAIN.drive(currentFwdCmd, 0.0, headingHoldOmega(), true);

      if (Math.abs(odomYIn - l4SegStartYInBack) >= Math.abs(backoffIn)) {
          // stop moving back
          DRIVETRAIN.drive(0,0,0,true);
          headingHoldStop();

          // === NEW: drop elevator back to home ===
          ELEVATOR.setHeight(l2HeightIn);

          l4Phase = L4Phase.DONE;
      }
      break;

    // finished
    case DONE:
    default:
      currentFwdCmd = 0.0;
      DRIVETRAIN.drive(0,0,0,true);
      break;
  }
}



  @Override
  public void teleopPeriodic() {
      // ===== One-shot wheel zero on enable =====
      if (doStartupWheelZero) {
          if (Timer.getFPGATimestamp() < startupWheelZeroUntilSec) {
              DRIVETRAIN.pointWheelsForward();
          } else {
              doStartupWheelZero = false;
          }
      }
  
      // ===== Elevator / Climber button mapping =====
      if (CONTROLLER.getCrossButtonPressed()) {
          ELEVATOR.setHeight(Constants.ELEVATOR_HEIGHTS[1]);
      } else if (CONTROLLER.getSquareButtonPressed()) {
          double h = Constants.ELEVATOR_HEIGHTS[2];
          if (ELEVATOR.inAlgaeMode()) h += 2.0;
          ELEVATOR.setHeight(h);
      } else if (CONTROLLER.getCircleButtonPressed()) {
          double h = Constants.ELEVATOR_HEIGHTS[3];
          if (ELEVATOR.inAlgaeMode()) h += 1.0;
          ELEVATOR.setHeight(h);
      } else if (CONTROLLER.getTriangleButtonPressed()) {
          double h = Constants.ELEVATOR_HEIGHTS[4];
          if (ELEVATOR.inAlgaeMode()) h += Constants.ALGAE_L4_OFFSET_IN;
          ELEVATOR.setHeight(h);
      } else if (CONTROLLER.getPOV() == 90) {
          // POV right = stow
          ELEVATOR.setHeight(Constants.ELEVATOR_HEIGHTS[0]);
      }
  
      if (CONTROLLER.getR2ButtonPressed()) {
          if (!ELEVATOR.inAlgaeMode()) {
              // coral score request
              ELEVATOR.ejectCoral();
          } else {
              // algae toggle spit/stop
              ELEVATOR.setEjection(!ELEVATOR.isEjecting());
          }
      }
  
      if (CONTROLLER.getL2ButtonPressed()) {
          // toggle algae mode
          ELEVATOR.setAlgaeMode(!ELEVATOR.inAlgaeMode());
      }
  
      // ===== Touchpad = reseed field-centric (make current facing "forward") =====
      if (CONTROLLER.getTouchpadButtonPressed()) {
          final double headingOffsetDeg = SmartDashboard.getNumber("Drive/HeadingOffsetDeg", 0.0);
          double yawCalib = SmartDashboard.getNumber("Drive/YawCalibDeg", 0.0);
          if (SmartDashboard.getBoolean("Drive/Apply90Fix", false)) {
              yawCalib += 90.0;
          }
  
          // 1. tell drivetrain "THIS direction is forward now"
          DRIVETRAIN.setDriverForwardOffsetDegrees(headingOffsetDeg + yawCalib);
  
          // 2. re-apply alliance perspective
          DRIVETRAIN.setOperatorPerspectiveForAlliance();
  
          // 3. lock into CTRE's field-centric transform
          DRIVETRAIN.seedFieldCentricNow();
  
          // 4. we're now normal field-oriented, so stop flipping forward/back
          SmartDashboard.putBoolean("Drive/ReverseFieldForward", false);
  
          System.out.println("[Drive] Touchpad reseed: field-centric reset & ReverseFieldForward=false");
      }
  
      // ===== Sticks → driver-frame commands =====
      double rawLX = CONTROLLER.getLeftX();
      double rawLY = CONTROLLER.getLeftY();
      double rawRX = CONTROLLER.getRightX();
  
      // shape -> slew filter
      double shapedStrafeRight = strafeLimiter.calculate(
          shapeInput(rawLX,  TRANS_DEADBAND, TRANS_EXPO)
      );
      double shapedForwardCmd = fwdLimiter.calculate(
          shapeInput(-rawLY, TRANS_DEADBAND, TRANS_EXPO)
      );
  
      // WPILib/CTRE convention: +X = fwd, +Y = left.
      double manualLeftCmd    = -shapedStrafeRight; // pushing stick right = robot should go right
      double manualForwardCmd =  shapedForwardCmd;
  
      // startup flip so forward stick can pull robot toward DS until reseed
      if (SmartDashboard.getBoolean("Drive/ReverseFieldForward", false)) {
          manualForwardCmd = -manualForwardCmd;
          manualLeftCmd    = -manualLeftCmd;
      }
  
      // rotation: driver stick right is CW, drivetrain wants +CCW
      double manualOmegaCCW = -rotLimiter.calculate(
          shapeInput(rawRX, ROT_DEADBAND, ROT_EXPO)
      ) * ROT_GAIN;
  
      if (SmartDashboard.getBoolean("Drive/InvertRotStick", false)) {
          manualOmegaCCW = -manualOmegaCCW;
      }
  
      // final commands (no Limelight override at all)
      double finalForwardCmd   = manualForwardCmd;
      double finalLeftCmd      = manualLeftCmd;
      double finalOmegaCCW     = manualOmegaCCW;
      boolean driveFieldOriented = true; // always field-oriented now
  
      // clean tiny noise
      double deadbandForSnap = 0.04;
      finalForwardCmd = snapZeroCustom(finalForwardCmd, deadbandForSnap);
      finalLeftCmd    = snapZeroCustom(finalLeftCmd,    deadbandForSnap);
      finalOmegaCCW   = snapZeroCustom(finalOmegaCCW,   deadbandForSnap);
  
      SmartDashboard.putNumber("Drive/OmegaCCW", finalOmegaCCW);
  
      boolean idle = (Math.abs(finalForwardCmd)  < IDLE_BAND &&
                      Math.abs(finalLeftCmd)     < IDLE_BAND &&
                      Math.abs(finalOmegaCCW)    < IDLE_BAND);
  
      if (idle) {
          DRIVETRAIN.drive(0, 0, 0, true);
      } else {
          DRIVETRAIN.drive(finalForwardCmd, finalLeftCmd, finalOmegaCCW, driveFieldOriented);
      }
  
      // ===== Climber quick controls =====
      int pov = CONTROLLER.getPOV();
      boolean downHeld  = (pov == 180);
      boolean upHeld    = (pov ==   0);
      boolean leftEdge  = (pov == 270) && (lastPOV != 270);
      if (leftEdge) {
          boolean hToggleActive = SmartDashboard.getBoolean("Climb/HToggleActive", false);
          hToggleActive = !hToggleActive;
          SmartDashboard.putBoolean("Climb/HToggleActive", hToggleActive);
          if (hToggleActive) {
              CLIMBER.runHPercent(SmartDashboard.getNumber("Climb/HTogglePercent", -0.5));
          } else {
              CLIMBER.stopH();
          }
      }

      // === dashboard color for climber H ===
// GREEN when spinning, RED when stopped
{
  boolean hActiveNow = SmartDashboard.getBoolean("Climb/HToggleActive", false);
  SmartDashboard.putString(
      "UI/ClimberHColor",
      hActiveNow ? "GREEN" : "RED"
  );
}

  
if (phase == Phase.IDLE) {
  double base = Math.abs(SmartDashboard.getNumber("Climb/ManualJogPercent",  0.14));
  double outM = Math.abs(SmartDashboard.getNumber("Climb/ManualJogOutMult",  3.0));
  double inM  = Math.abs(SmartDashboard.getNumber("Climb/ManualJogInMult",   3.0));
  double jogOut = Math.min(1.0, base * outM);
  double jogIn  = Math.min(1.0, base * inM);

  if (upHeld ^ downHeld) {
      if (upHeld)  {
          CLIMBER.runGBPercent(-homeDirSign * jogOut);
      } else {
          CLIMBER.runGBPercent(+homeDirSign * jogIn);
      }
  } else {
      CLIMBER.stopGB();
  }
}

    // === UPDATE DRIVERUI BIG COLOR BOXES ===
    // algaeModeBoxEntry:
    //   true  (Lime)  => ALGAE MODE
    //   false (Red)   => CORAL MODE
    boolean algaeIsOn = ELEVATOR.inAlgaeMode();
    if (algaeModeBoxEntry != null) {
        algaeModeBoxEntry.setBoolean(algaeIsOn);
    }

    // climberHBoxEntry:
    //   true  (Lime)  => H is actively spinning
    //   false (Red)   => H is not spinning
    boolean hActiveNow = SmartDashboard.getBoolean("Climb/HToggleActive", false);
    if (climberHBoxEntry != null) {
        climberHBoxEntry.setBoolean(hActiveNow);
    }

    lastPOV = pov;
}


  

  @Override
  public void disabledInit() {
    DRIVETRAIN.drive(0,0,0,false);
    if (CLIMBER != null) CLIMBER.stopAll();

    LimelightHelpers.setLEDMode_PipelineControl(LL_NAME);
    ALIGN.disable();
    SmartDashboard.putBoolean("LL/AlignActive", false);
  }
  @Override public void disabledPeriodic() {}
  @Override public void testInit() {}
  @Override public void testPeriodic() {}

  private static double shapeInput(double raw, double deadband, double expo) {
    double v = MathUtil.applyDeadband(raw, deadband);
    return Math.copySign(Math.pow(Math.abs(v), expo), v);
  }
  private static double snapZero(double v) {
    return (Math.abs(v) < 0.04) ? 0.0 : v;
  }
  private static double snapZeroCustom(double v, double band) {
    return (Math.abs(v) < band) ? 0.0 : v;
}
}
