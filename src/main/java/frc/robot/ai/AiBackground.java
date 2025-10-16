
package frc.robot.ai;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Background driving assists (NO auto-align / NO vision control).
 * Toned-down + smoothed so high-speed driving feels normal.
 *
 * Features (all blended & rate-friendly):
 *  - Brownout softening: small XY scale near low voltage (6.8-7.2V band).
 *  - Tip/bumps guard: mild XY scale if |pitch| or |dPitch/dt| spikes.
 *  - Micro yaw-damping: light ω correction only at low/med speeds & when driver isn't turning.
 *  - Precision micro-scale for tiny sticks.
 *  - Watchdog: brief, gentle safe-mode after big yaw jolt or deep dip.
 *
 * All outputs are [-1..1] like your shaped commands.
 */
public class AiBackground {

    public static class Output {
        public final double x, y, omega;
        public Output(double x, double y, double omega) { this.x = x; this.y = y; this.omega = omega; }
    }

    // ---- sensor suppliers provided by Robot.java ----
    private final Supplier<Double> yawDeg;
    private final Supplier<Double> yawRateDps;   // CorePigeon2.getAngularVelocityZWorld().getValueAsDouble()
    private final Supplier<Double> pitchDeg;     // CorePigeon2.getPitch().getValueAsDouble()

    public AiBackground(Supplier<Double> yawDeg,
                        Supplier<Double> yawRateDps,
                        Supplier<Double> pitchDeg) {
        this.yawDeg = yawDeg;
        this.yawRateDps = yawRateDps;
        this.pitchDeg = pitchDeg;
    }

    // ---- state ----
    private double lastV = RobotController.getBatteryVoltage();
    private double lastPitch = 0.0;
    private double lastT = Timer.getFPGATimestamp();
    private double safeModeUntil = 0.0; // timestamp when watchdog safe-mode ends

    // smoothing state (low-pass on scaling & ω correction)
    private double smoothTransScale = 1.0;
    private double smoothedYawRate = 0.0;

    public void initTunableDefaults() {
        // Master controls
        SmartDashboard.putBoolean("AIbg/Enable", true);
        SmartDashboard.putNumber("AIbg/AssistGain", 0.45);        // 0..1 blend amount (lower = softer)
        SmartDashboard.putNumber("AIbg/ScaleLPAlpha", 0.25);      // 0..1 smoothing for scale changes

        // Brownout softening (refs: roboRIO brownout docs)
        SmartDashboard.putNumber("AIbg/VbrownSoft", 7.20);        // start scaling below this
        SmartDashboard.putNumber("AIbg/VbrownHard", 6.80);        // heavy-ish scaling here
        SmartDashboard.putNumber("AIbg/ScaleBrownMin", 0.85);     // never scale below this (toned down)

        // Tip/bumps guard (mild)
        SmartDashboard.putNumber("AIbg/TipPitchDeg", 8.5);        // deg
        SmartDashboard.putNumber("AIbg/TipPitchRate", 60.0);      // deg/s
        SmartDashboard.putNumber("AIbg/ScaleTip", 0.90);          // mild scale when triggered
        SmartDashboard.putNumber("AIbg/TipHoldMs", 200.0);        // small hysteresis hold

        // Micro yaw-damping (very light, only when not turning & not at max speed)
        SmartDashboard.putNumber("AIbg/YawDpsToCmd", 0.008);      // cmd per deg/s (reduced gain)
        SmartDashboard.putNumber("AIbg/YawDampMax", 0.12);        // clamp on add
        SmartDashboard.putNumber("AIbg/YawDeadband", 0.08);       // consider driver turning above this ω
        SmartDashboard.putNumber("AIbg/YawRateLPAlpha", 0.20);    // low-pass for noisier IMU rates

        // Precision tiny-stick scaling
        SmartDashboard.putBoolean("AIbg/PrecisionEnable", true);
        SmartDashboard.putNumber("AIbg/PrecisionStickMag", 0.16);
        SmartDashboard.putNumber("AIbg/PrecisionScale", 0.75);

        // Watchdog (gentle)
        SmartDashboard.putNumber("AIbg/JoltYawDps", 300.0);
        SmartDashboard.putNumber("AIbg/SafeMs", 250.0);
        SmartDashboard.putNumber("AIbg/SafeScale", 0.80);

        // Debug
        SmartDashboard.putBoolean("AIbg/Debug", false);
    }

    public void onTeleopInit() {
        lastV = RobotController.getBatteryVoltage();
        lastPitch = pitchDeg.get();
        lastT = Timer.getFPGATimestamp();
        safeModeUntil = 0.0;
        smoothTransScale = 1.0;
        smoothedYawRate = 0.0;
    }

    public Output apply(double xCmd, double yCmd, double omegaCmd,
                        boolean driverTurning, boolean driverTranslating) {

        if (!SmartDashboard.getBoolean("AIbg/Enable", true)) {
            return new Output(xCmd, yCmd, omegaCmd);
        }

        final double now = Timer.getFPGATimestamp();
        final double dt  = Math.max(1e-3, now - lastT);

        // ---- read sensors ----
        final double V   = RobotController.getBatteryVoltage();
        final double dV  = (V - lastV) / dt;               // V/s
        final double pitch = pitchDeg.get();
        final double dPitch = (pitch - lastPitch) / dt;    // deg/s
        final double yawDpsMeas = yawRateDps.get();

        // LP filter the yaw rate (reduce noise / jitter at speed)
        double yawLP = MathUtil.clamp(SmartDashboard.getNumber("AIbg/YawRateLPAlpha", 0.20), 0.0, 1.0);
        smoothedYawRate += yawLP * (yawDpsMeas - smoothedYawRate);
        final double yawDps = smoothedYawRate;

        // ---- watchdog safe-mode (big yaw jolt OR deep dip)
        final double joltThresh = SmartDashboard.getNumber("AIbg/JoltYawDps", 300.0);
        final double safeMs     = SmartDashboard.getNumber("AIbg/SafeMs", 250.0);
        if ((!driverTurning && Math.abs(yawDps) > joltThresh) ||
            V < SmartDashboard.getNumber("AIbg/VbrownHard", 6.80)) {
            safeModeUntil = now + safeMs / 1000.0;
        }
        final boolean inSafeMode = now < safeModeUntil;

        // ---- brownout predictor scaling (very mild & smoothed)
        double scaleBrown = 1.0;
        double Vsoft = SmartDashboard.getNumber("AIbg/VbrownSoft", 7.20);
        double Vhard = SmartDashboard.getNumber("AIbg/VbrownHard", 6.80);
        double minBrown = MathUtil.clamp(SmartDashboard.getNumber("AIbg/ScaleBrownMin", 0.85), 0.70, 1.0);

        if (V < Vsoft) {
            // map [Vhard..Vsoft] -> [minBrown..1.0]
            double t = MathUtil.clamp((V - Vhard) / Math.max(0.01, (Vsoft - Vhard)), 0.0, 1.0);
            scaleBrown = minBrown + (1.0 - minBrown) * t;
        }
        // small pre-emptive nudge if voltage is dropping very fast under heavy stick
        double stickMag = Math.hypot(xCmd, yCmd);
        if (dV < -1.2 && stickMag > 0.6) { // rapid drop & aggressive driving
            scaleBrown = Math.min(scaleBrown, Math.max(minBrown, 0.92));
        }

        // ---- tip/bumps guard scaling (mild, with hysteresis)
        double scaleTip = 1.0;
        double pAbs = Math.abs(pitch);
        double pRateAbs = Math.abs(dPitch);
        double pTh  = SmartDashboard.getNumber("AIbg/TipPitchDeg", 8.5);
        double pRth = SmartDashboard.getNumber("AIbg/TipPitchRate", 60.0);

        final boolean tipTriggered = (pAbs > pTh) || (pRateAbs > pRth);
        final double tipHoldMs = SmartDashboard.getNumber("AIbg/TipHoldMs", 200.0);
        // reuse safeModeUntil lightly for a tiny hold if tipping just started
        if (tipTriggered) {
            safeModeUntil = Math.max(safeModeUntil, now + tipHoldMs / 1000.0);
        }
        if (tipTriggered) {
            scaleTip = MathUtil.clamp(SmartDashboard.getNumber("AIbg/ScaleTip", 0.90), 0.75, 1.0);
        }

        // ---- base translational scale target, smoothed
        double targetTransScale = Math.min(scaleBrown, scaleTip);
        if (inSafeMode) {
            targetTransScale = Math.min(targetTransScale, MathUtil.clamp(SmartDashboard.getNumber("AIbg/SafeScale", 0.80), 0.6, 1.0));
        }

        // smooth changes so we never "snap" the feel at speed
        double aScale = MathUtil.clamp(SmartDashboard.getNumber("AIbg/ScaleLPAlpha", 0.25), 0.0, 1.0);
        smoothTransScale += aScale * (targetTransScale - smoothTransScale);

        // ---- yaw micro-damping (light & only when not turning fast)
        double omegaOut = omegaCmd;
        final boolean allowYawDamp = !inSafeMode
                && !driverTurning
                && stickMag < 0.7; // fade out at high translational speeds

        if (allowYawDamp) {
            double gain  = SmartDashboard.getNumber("AIbg/YawDpsToCmd", 0.008);
            double maxAdd = SmartDashboard.getNumber("AIbg/YawDampMax", 0.12);
            // fade the correction to zero as stick approaches 0.7..1.0
            double fade = MathUtil.clamp(1.0 - MathUtil.interpolate(0.0, 1.0, (stickMag - 0.4) / 0.6), 0.0, 1.0);
            double add = MathUtil.clamp(-gain * yawDps * fade, -maxAdd, +maxAdd);
            omegaOut = MathUtil.clamp(omegaOut + add, -1.0, 1.0);
        }

        // ---- precision mode for tiny stick motions (mild)
        double xAssist = xCmd;
        double yAssist = yCmd;
        if (SmartDashboard.getBoolean("AIbg/PrecisionEnable", true)
                && !inSafeMode
                && stickMag < SmartDashboard.getNumber("AIbg/PrecisionStickMag", 0.16)) {
            double s = SmartDashboard.getNumber("AIbg/PrecisionScale", 0.75);
            xAssist *= s;
            yAssist *= s;
        }

        // ---- apply translational scale (smoothed), then blend with original cmds
        xAssist *= smoothTransScale;
        yAssist *= smoothTransScale;

        // final blend: original vs assisted (keeps assists subtle)
        double k = MathUtil.clamp(SmartDashboard.getNumber("AIbg/AssistGain", 0.45), 0.0, 1.0);
        double xOut = (1.0 - k) * xCmd + k * xAssist;
        double yOut = (1.0 - k) * yCmd + k * yAssist;
        double omegaFinal = omegaOut; // omega kept separate (small additive only)

        // ---- debug ----
        if (SmartDashboard.getBoolean("AIbg/Debug", false)) {
            SmartDashboard.putNumber("AIbg/V", V);
            SmartDashboard.putNumber("AIbg/dVdt", dV);
            SmartDashboard.putNumber("AIbg/pitch", pitch);
            SmartDashboard.putNumber("AIbg/dPitch", dPitch);
            SmartDashboard.putNumber("AIbg/yawDpsLP", yawDps);
            SmartDashboard.putBoolean("AIbg/SafeMode", inSafeMode);
            SmartDashboard.putNumber("AIbg/ScaleTrans", smoothTransScale);
            SmartDashboard.putNumber("AIbg/StickMag", stickMag);
        }

        // ---- update history ----
        lastT = now;
        lastV = V;
        lastPitch = pitch;

        return new Output(
            MathUtil.clamp(xOut, -1.0, 1.0),
            MathUtil.clamp(yOut, -1.0, 1.0),
            MathUtil.clamp(omegaFinal, -1.0, 1.0)
        );
    }
}

