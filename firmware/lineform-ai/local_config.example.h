// Per-arm settings - copy this file to local_config.h (which is git-ignored)
// and edit it for your own arm. local_config.h is never uploaded to GitHub.

// GUI motor numbers (1 = base ... 11 = tip, same as the slider labels) of
// motors mounted facing the other way, comma-separated. Test by moving each
// slider: any motor that bends the opposite way from the others goes here.
// Leave empty if none are reversed.
#define REVERSED_MOTORS_LIST  /* e.g. 3, 7 */

// How many motors this arm actually has, base -> tip (the first N entries
// of DXL_IDS). Leave commented out for the full 11.
// #define ACTIVE_MOTORS  10

// Angle correction per motor number (1..11), degrees: what the GUI shows
// minus the real angle. Missing entries count as 0.
// #define ANGLE_OFFSETS_DEG  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

// Move speed limits (0 = unlimited). Defaults: 150 / 40.
// #define MOVE_PROFILE_VELOCITY      150
// #define MOVE_PROFILE_ACCELERATION  40

// Use the backup firmware's plain position mode (torque sliders then have
// no effect). Try this if motors keep tripping the overload error.
// #define USE_PLAIN_POSITION_MODE

// Position I Gain (removes gravity droop; 0 = off). Default 150; lower if motors twitch.
// #define DROOP_I_GAIN  150
