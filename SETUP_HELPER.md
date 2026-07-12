# Engineering Workspace Setup Guide (v2.5)

This helper guide details the step-by-step setup procedure to configure your local engineering workspace for Version 2.5 of the **decode** FTC Robot Controller.

---

## 1. System Requirements & Toolchain Conventions

* **JDK (Java Development Kit)**: **Java 21** is required.
* **Build System**: **Gradle 8+** (handled automatically via the wrapper `./gradlew`).
* **Toolchain Resolver**: The project uses the Foojay resolver convention (`settings.gradle`) to automatically configure the toolchain.

---

## 2. Setup & Synchronization Steps

Follow these steps sequentially to configure your local environment:

### Step A: Clean Previous Build Artifacts
Ensure no stale caching files interfere with the new Sloth and Dairy build structures:
```bash
./gradlew clean
```

### Step B: Sync and Compile Gradle Dependencies
Verify that all Maven dependencies (including dev.frozenmilk.dairy, com.pedropathing, and YAML parsers) load correctly:
```bash
./gradlew compileDebugSources
```
*(In Android Studio, click **File > Sync Project with Gradle Files**)*

### Step C: Verify Classpath Configuration Files
Make sure the runtime configuration files are in place under:
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/robot/config/`
* `subsystems.yaml`
* `safety.yaml`
* `tuning.yaml`

---

## 3. Managing and Rebuilding Engineering Rules

If you modify coding conventions, mechanical rules, or AI guidelines, follow this procedure to rebuild the sharded rule files for our AI assistants (Claude, Copilot, Codex, Antigravity):

1. **Source Location**: Modify rules under the source folder `.ai-rulez/`:
   - System Rules: `.ai-rulez/rules/code-quality.md`
   - System Context: `.ai-rulez/context/architecture.md`
   - System Configuration: `.ai-rulez/config.toml`

2. **Trigger Compilation**:
   Run the `ai-rulez` generation tool to update target sharded files:
   ```bash
   npx ai-rulez@latest generate
   ```

3. **Verify Generated Files**:
   Ensure downstream files (which are ignored by Git via `.gitignore`) compile correctly:
   - `AGENTS.md` (Antigravity preset)
   - `CLAUDE.md` (Claude preset)
   - `.github/copilot-instructions.md` (Copilot preset)
   - `.codex/` (Codex preset)

4. **Verify Configurations**:
   You can run validation to test for configuration layout errors:
   ```bash
   npx ai-rulez@latest validate
   ```
