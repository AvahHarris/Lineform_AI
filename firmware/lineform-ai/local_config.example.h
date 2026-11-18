// Per-arm settings - copy this file to local_config.h (which is git-ignored)
// and edit it for your own arm. local_config.h is never uploaded to GitHub.

// Hardware IDs (= the GUI labels, 2 = base ... 12 = tip) of motors mounted
// facing the other way, comma-separated. Test by moving each
// slider: any motor that bends the opposite way from the others goes here.
// Leave empty if none are reversed.
#define REVERSED_MOTORS_LIST  /* e.g. 3, 7 */

// How many motors this arm actually has, base -> tip (the first N entries
// of DXL_IDS = IDs 2..12). Leave commented out for the full 11.
// #define ACTIVE_MOTORS  10

// Angle correction per motor, degrees: what the GUI shows minus the real
// angle. Chain order, base -> tip (first entry = ID 2 ... last = ID 12).
// Missing entries count as 0.
// #define ANGLE_OFFSETS_DEG  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

// Move speed limits (0 = unlimited). Defaults: 150 / 40.
// #define MOVE_PROFILE_VELOCITY      150
// #define MOVE_PROFILE_ACCELERATION  40

// Use the backup firmware's plain position mode (torque sliders then have
// no effect). Try this if motors keep tripping the overload error.
// #define USE_PLAIN_POSITION_MODE

// ---- Stiffness (depends on the motor MODEL - works for mixed chains) ----
// Each motor reports its model number (the GUI's startup diagnostics list
// them, e.g. XL330-M077 = 1190, XL330-M288 = 1200) and automatically uses
// the gain set for its model. One set per model, one per line:
//   { model, P level 0, P level 1, P level 2, P level 3, I gain at level 0 }
// Level 0 = normal holding; 1-3 = softer (the STIFF bars, animations, the
// GUI's twitch safety net). The I gain removes gravity droop at level 0.
// Low-gear-ratio motors (XL330-M077, factory P 400) may twitch at high gains;
// higher-gear-ratio ones (XL330-M288) tolerate more. Starting points:
// #define MODEL_GAINS_LIST \
//   {1190,  800,  700, 350, 120,  30},  /* XL330-M077 (soft) */ \
//   {1200, 1800, 1200, 700, 300, 150}   /* XL330-M288 (stiff) */

// Fallback for any motor whose model has no set above (and the only setting
// needed on an arm with a single model). Defaults: 1200, 700, 350, 120 / 150.
// #define STIFFNESS_P_GAINS_LIST  1200, 700, 350, 120
// #define DROOP_I_GAIN  150

// Per-motor trim, % of its model's set, chain order base -> tip (IDs 2..12;
// missing / 0 = 100%). For a single motor that twitches more than others of its model.
// #define P_GAIN_PCT_LIST  100, 100, 100, 100, 100, 50, 50

// Force a motor onto another model's set (model number; 0 = what it reports),
// chain order base -> tip (IDs 2..12) - e.g. if a motor's model can't be read.
// #define MOTOR_MODEL_OVERRIDE_LIST  0, 0, 0, 0, 1200
