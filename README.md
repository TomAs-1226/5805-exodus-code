
# 2025 Exodus Code — FRC Reefscape (Java / WPILib 2025)

Robotics codebase for **Reefscape 2025**, built with **Java + WPILib 2025** and **GradleRIO**.  
Includes a swerve drivetrain (field-oriented with Pigeon2), elevator subsystem, climber, PathPlanner autos (L2 → L4 sequencing), Limelight aiming hooks, and a Shuffleboard auto chooser.

---

## ✨ Highlights

- **Swerve Drive** (field-oriented) using **Pigeon2** gyro  
- **Elevator** with staged presets and **coral eject** control
- **Climber** with both **auto-climb toggle** and **manual (hold)**
- **PathPlanner** autos including:
  - *OnePiece-L2-Then-L4*: raise to **L2**, drive to Reef, score, **back off 4 in**, raise to **L4**, eject, depart avoiding the **Barge** path
- **PS5 Controller** mapping (as requested):
  - **D-pad Down (hold)** → bring mechanism **in** (used for intake/arm/climber “in”)
  - **D-pad Left (press)** → **toggle Auto-Climb**
- **Shuffleboard/SmartDashboard** auto chooser
- **Limelight** helpers available for auto-aim (optional)
- Clean **.gitignore**: no `build/`, no `*.pdb` or compiled artifacts in Git

---

## 🧱 Project Structure


preseason-2025/
├─ src/main/java/frc/robot/
│  ├─ Robot.java
│  ├─ Constants.java
│  ├─ command/...
│  ├─ subsystem/
│  │  ├─ drive/SwerveDrivetrain.java
│  │  ├─ elevator/Elevator.java
│  │  └─ ClimberSubsystem.java
│  ├─ util/RobotMath.java
│  └─ vision/LimelightHelpers.java
├─ src/main/deploy/
│  └─ pathplanner/                  # *.path & *.trajectory files
├─ vendordeps/
│  ├─ phoenix6.json                 # CTRE (Kraken/Pigeon2)
│  └─ revlib.json                   # REV (if used)
├─ build.gradle                     # GradleRIO config
├─ gradlew / gradlew.bat / gradle/
└─ README.md



---

## 🧰 Tech Stack

- **Language:** Java 17 (WPILib 2025)
- **Build:** GradleRIO (VS Code WPILib plugin)
- **Motor/IO:** CTRE **Phoenix 6** (Kraken X60/X44, Pigeon2)
- **Vision (optional):** Limelight (NT/AprilTag)
- **Autos:** **PathPlanner 2025** (event markers supported)
- **Dashboard:** Shuffleboard / SmartDashboard

> Note: We also discussed CAN IDs (e.g., 52/50) for Kraken motors and Pigeon2; please update `Constants` to match your robot’s actual wiring.

---

## 🚀 Getting Started

1. **Install** WPILib 2025 + VS Code extension (Java 17).  
2. **Vendors:** Put `phoenix6.json` (and `revlib.json` if needed) into `vendordeps/` or add via **Manage Vendor Libraries**.
3. **Team number:** Set it in **WPILib VS Code** (or the project’s `wpilib_preferences.json`).
4. **Build & Deploy:**
   ```bash
   ./gradlew build
   ./gradlew deploy

5. **Driver Station:** Enable & test in a safe space (bumpers on, blocks up).

---

## 🗺️ PathPlanner Setup

* **Files:** Place `.path`/`.trajectory` under `src/main/deploy/pathplanner/`.
* **Chooser:** Robot code exposes an **Auto Chooser** on Shuffleboard.
* **One-Piece L4 Routine (what we built):**

  1. **Raise to L2** (safer clearance while driving)
  2. Drive to **Reef** along planned path
  3. **Score** coral
  4. **Back off 4 inches** (to clear)
  5. **Raise to L4** and **eject**
  6. **Leave lane** with offsets to avoid the **Barge**

> **Naming tip:** Keep autos semantic, e.g., `OnePiece_L2ThenL4_Back4in`.
> **Event markers** can call commands (elevator setpoint, eject, etc.) at precise path points.

---

## 🎮 Controls (PS5)

* **Left stick:** X/Y translation
* **Right stick:** rotation
* **D-pad Down (hold):** bring mechanism **in** (we mapped this per your request)
* **D-pad Left (press):** **toggle Auto-Climb**
* **Bumpers/Triggers/Face buttons:** map to elevator presets, eject, slow mode, etc. (see `Robot.java` / `CommandSystem`)

Update `Constants` if you rebind buttons.

---

## ⚙️ Tuning & Known Fixes

### Swerve “tiny skew” when strafing

* Verify **absolute encoder zeros** for each module (mechanical alignment).
* Re-seed **AZIMUTH OFFSETS** in `Constants`.
* Check **Pigeon2** yaw health and **field-orientation** toggle.
* Use **deadband** + **SlewRateLimiter** on sticks to smooth micro-inputs.
* Confirm the **kinematics center** & sign conventions (left/right inversion).

### “`edu.wpi.first.units.Angle` cannot be resolved”

* Ensure WPILib **2025** is installed in your project.
* If you’re using Phoenix6 **StatusSignal** with units, you can:

  * Use the units-typed form (requires WPILib units packages), **or**
  * Fall back to `StatusSignal<Double>` and treat values as **degrees** (`getValue()`).

### Git/GitHub large-file rejection (✅ fixed here)

* Do **not** commit `build/` outputs or `*.pdb` files.
* This repo is clean; `.gitignore` keeps them out.
* If you truly need binaries, configure **Git LFS** (not recommended for build outputs).

---

## 🔧 Configuration Notes

* **CAN IDs:** Document in `Constants` (e.g., `DRIVE_FL_ID`, `ELEVATOR_MASTER_ID`, `PIGEON2_ID`).
* **Subsystem presets:**

  * `Elevator`: `L2`, `L3`, `L4` setpoints with motion constraints.
  * `Shooter/Eject`: power per level (e.g., `EjectPower.L2`, `EjectPower.L4`).
* **Auto parameters:**

  * Back-off distance default: **4 in** (change in constants or auto command).
  * Barge avoidance: path XY offsets in PathPlanner.

---

## 🧪 Developer Workflow

```bash
# install vendors (if not already)
# WPILib VS Code: Manage Vendor Libraries → Install new (JSON / online)

# build & run tests (if any)
./gradlew build

# deploy to roboRIO
./gradlew deploy

# logs
./gradlew rioLog
```

**Branching:** `main` (stable) + `feature/*` branches for new subsystems/auto paths.
**Commits:** Use clear, present-tense messages (e.g., `tune: L4 eject power`, `feat: add L2→Reef→L4 auto`).

---

## 📁 .gitignore (summary)

We keep these out of git:

```
build/ **/build/ .gradle/ out/ bin/
*.pdb *.exe *.dll
.idea/ .vscode/ *.iml
.DS_Store Thumbs.db
.wpilib/active.json
```

> Keep **`gradle/wrapper/**`**, **`gradlew`**, and **`vendordeps/*.json`** **tracked**.

---

## 🤝 Contributing

1. Create a branch: `git checkout -b feature/short-name`
2. Commit changes: `git commit -m "feat: what you did"`
3. Push: `git push -u origin feature/short-name`
4. Open a **Pull Request** with screenshots/notes (paths, setpoints, controller bindings)

---

## 📜 License

Choose a license (MIT/BSD-3/Apache-2.0). Add it as `LICENSE` in the repo root.

---

## 🙏 Acknowledgements

* WPILib maintainers & PathPlanner team
* CTRE Phoenix / Kraken & Pigeon2
* Everyone who tested autos and tuned the L2 → L4 sequence (and measured the **4-inch** back-off!)

---

## ✅ TODO (next passes)

* [ ] Finalize elevator setpoints for **L2/L3/L4** and safe interlocks
* [ ] Tune eject powers per level (especially **L4**)
* [ ] Add AprilTag alignment command (Limelight or PhotonVision)
* [ ] Add additional autos for different start locations
* [ ] Record CAN IDs & wiring in `Constants` + README table

```
```
