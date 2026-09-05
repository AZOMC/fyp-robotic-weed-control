/* ==================================================================
   WEED ROBOT — MASTER FIRMWARE
   Merge of: Robot_Movement.ino + delta_robot_ik.ino + Gripper_Test.ino
   Board : Arduino Mega 2560 + RAMPS 1.6 + DRV8825 @ 1/32
   BT    : HC-05 on Serial1 (RX1=D19, TX1=D18) @ 9600
   USB   : 115200  <-- set your Serial Monitor to 115200, not 9600

   ------------------------------------------------------------------
   WHAT THIS FIRMWARE DOES
   ------------------------------------------------------------------
   Autonomous cycle, triggered by one line from the Raspberry Pi:

       PICK X50 Y20        (Z defaults to pickZ)

     1. stop the drive wheels (safety)
     2. open gripper
     3. move to (X, Y, TRANSIT_Z)         travel feed, sideways leg
     4. descend to (X, Y, Z)              half feed, vertical only
     5. CLOSE gripper
     6. lift to (X, Y, TRANSIT_Z)         half feed
     7. move to (BIN_X, BIN_Y, BIN_Z)     travel feed, flat sideways leg
     8. OPEN gripper -- the weed drops into the bin
     9. print  DONE  immediately
    10. return to the working position (best effort)

   Every sideways leg is flown at TRANSIT_Z, which sits above the bin
   keep-out plane, so no XY move can ever clip the bin. BIN_Z equals
   TRANSIT_Z, so there is no descent into the bin and no lift back out.

   DONE is sent at step 9, BEFORE the arm travels home. The Pi may
   send the next detection straight away: bytes arriving during the
   return leg are parked and replayed when it finishes, so the next
   PICK begins as soon as the arm is free -- no re-homing, no waiting
   for it to reach the working position.

   On any failure it prints  ERR <reason>  instead of DONE.
   The Pi MUST wait for DONE (or ERR) before sending the next
   detection — see "PI HANDSHAKE" note at the bottom of this header.

   ------------------------------------------------------------------
   PIN MAP  (no two subsystems share a pin — verified)
   ------------------------------------------------------------------
   DELTA STEPPERS (RAMPS)
     X: STEP 54  DIR 55  EN 38
     Y: STEP 60  DIR 61  EN 56
     Z: STEP 46  DIR 48  EN 62

   DRIVE MOTORS (2x L298N)
     LEFT   ENA D2   ENB D3   IN1 D25  IN2 D23  IN3 D17  IN4 D16
     RIGHT  ENA D6   ENB D11  IN1 D32  IN2 D47  IN3 D45  IN4 D43

   GRIPPER
     Servo signal D4
     Servo V+     EXTERNAL 5-6V supply (NEVER the RAMPS 5V pin)
     Servo GND    common with Mega ground AND external supply ground

   NEVER put anything on D0/D1 — that is the bootloader's upload path.

   ------------------------------------------------------------------
   TIMER / PWM NOTES  (why this combination is safe)
   ------------------------------------------------------------------
   * Servo.h on the Mega grabs Timer5 for the first 12 servos.
     Timer5 owns pins 44, 45, 46.
       - D46 is Z_STEP, driven by direct PORTL writes, not analogWrite
         -> unaffected.
       - D45 is R_IN3, digitalWrite only -> unaffected.
     Nothing here calls analogWrite() on 44/45/46, so there is no clash.
   * Drive PWM uses D2/D3 (Timer3) and D6 (Timer4) / D11 (Timer1).
     Servo does not touch those timers while only one servo is attached.
   * D16/D17 are Serial2's TX2/RX2. Serial2 is never started here, so
     they are free as plain digital outputs for L_IN4 / L_IN3.

   ------------------------------------------------------------------
   COMMANDS
   ------------------------------------------------------------------
   DRIVE (single character, no newline needed — phone app friendly)
     F  forward     B  backward
     R  pivot right L  pivot left
     T  speed up    X  speed down
     0  STOP everything (wheels + abort delta move)

   AUTONOMOUS
     PICK X<n> Y<n> Z<n>   full pick-and-bin cycle, replies DONE
     AP1 / AP0             auto-pick: make a bare "X.. Y.. Z.." line
                           run the PICK cycle instead of a plain move
     WORK                  go to working position
     BIN                   go to bin position

   DELTA MOTION (unchanged)
     X<n> Y<n> Z<n> [F<n>]  plain coordinated move
     REACH X Y Z            reachability check, no motion
     WEED  X Y Z            approach / grip / retract (now grips for real)
     HOME  PARK  CIRCLE  SQUARE
     ABS | REL   POS  LIMITS  HELP

   FOR THE PI WEB PANEL (demo.py)   -- added, nothing else changed
     STATUS                 one parsable line of key=value machine state,
                            prefixed "ST ". Safe to poll ~1 Hz.
     JOGX<n> JOGY<n> JOGZ<n>
                            one-shot RELATIVE Cartesian jog in mm that
                            does not touch ABS/REL mode. Replies with
                            exactly one of "OK JOG X.. Y.. Z.." or
                            "ERR BUSY" / "ERR NOTHOMED" / "ERR JOG", so
                            the Pi can gate on the reply the same way it
                            gates on DONE after a PICK.

   POWER / FREE MOVEMENT
     D       release EVERYTHING: steppers off + servo detached, and
             the home reference is CLEARED (arms can now be moved by
             hand, so the stored position is no longer trustworthy).
             Send SETHOME again after repositioning.
     E       re-enable steppers + re-attach servo
     DS / ES steppers only, home reference kept (use for cooling)
     GDET / GATT   servo only

   GRIPPER (all G-prefixed to avoid clashing with drive keys)
     GO      open preset          GC      close preset
     GP      nudge open           GM      nudge close
     GA<n>   go to angle n        GSWEEP  slow sweep (find limits)
     GDET    detach               GATT    attach
     GOP<n>  set OPEN angle       GCL<n>  set CLOSE angle
     GMIN<n> set MIN angle        GMAX<n> set MAX angle
     GDWL<n> grip dwell ms        GAD<n>  auto-detach when idle 1/0

   CONFIG
     SETHOME  UNHOME   GR<n>  MS<n>  MAP0..5  IDENT  IX IY IZ
     SX/SY/SZ<n> raw steps   CALX/CALY/CALZ<n>
     TMIN TMAX RMAX ZMIN ZMAX  ACC<n>  SEG<n>
     BX<n> BY<n> BZ<n>   bin position      SETBIN   (capture current)
     WX<n> WY<n> WZ<n>   working position  SETWORK  (capture current)
     KPX<n> KPZ<n>       bin keep-out volume
     TRZ<n>              transit Z: height all XY travel happens at
     PZ<n>               default weed depth when PICK omits Z
     JF JB JR JL JT JX   raw single-motor jog (unhomed only)

   ------------------------------------------------------------------
   PI HANDSHAKE
   ------------------------------------------------------------------
   While a delta move runs the main loop is BLOCKED, but bytes are no
   longer lost: the abort poll parks them and loop() replays them when
   the move ends. production.py should still gate on the reply so it
   never gets ahead of the arm:
       ser.write(b"PICK X%.1f Y%.1f\n" % (x, y))   # Z defaults to pickZ
       wait for a line == "DONE" (or startswith "ERR")
   DONE arrives as the weed is released, so the next PICK can be sent
   while the arm is still returning home. A PICK that arrives while
   one is genuinely still running is answered with ERR BUSY.

   PLATFORM ADVANCE (demo.py autoweeding)
   --------------------------------------
   When the Pi sees no reachable weed for a few seconds it drives the
   base with plain "F" and refreshes it about every 1.2 s -- inside the
   FAILSAFE_TIMEOUT_MS window above, so if the Pi dies, the browser tab
   closes, or Bluetooth drops, the wheels stop on their own within 5 s.

   The moment a reachable weed appears the Pi sends "0" ITSELF, waits
   for the platform to settle, and only then re-measures the weed and
   sends PICK. doPick() still calls stopAll() on arrival, but by then
   the wheels are already stopped: doing it from the Pi means the
   coordinates that get picked were measured with the base standing
   still, instead of being a frame or two stale from the roll.
   ================================================================== */

#include <math.h>
#include <Servo.h>

// ==================================================================
//  TYPES
// ==================================================================
enum IkErr : uint8_t {
  IK_OK = 0,
  IK_UNREACH,   // discriminant < 0 (rods can't close)
  IK_ANG_LO,    // arm above thetaMin
  IK_ANG_HI,    // arm below thetaMax
  IK_CYL,       // outside soft cylinder
  IK_BINZONE    // would drive into the bin
};

struct Token {
  char    alpha[8];
  uint8_t alen;
  float   val;
  bool    hasVal;
};

// ==================================================================
//  RAMPS 1.6 PIN MAP  (delta steppers)
// ==================================================================
#define X_STEP_PIN  54   // PF0
#define X_DIR_PIN   55
#define X_EN_PIN    38
#define Y_STEP_PIN  60   // PF6
#define Y_DIR_PIN   61
#define Y_EN_PIN    56
#define Z_STEP_PIN  46   // PL3
#define Z_DIR_PIN   48
#define Z_EN_PIN    62

// Direct port masks — eliminates digitalWrite jitter on step pins
const uint8_t STEP_F_MASK[3] = { 0x01, 0x40, 0x00 };  // PORTF bits for X,Y
const uint8_t STEP_L_MASK[3] = { 0x00, 0x00, 0x08 };  // PORTL bit  for Z
const uint8_t DIR_PIN[3]     = { X_DIR_PIN, Y_DIR_PIN, Z_DIR_PIN };
const uint8_t EN_PIN[3]      = { X_EN_PIN,  Y_EN_PIN,  Z_EN_PIN  };
const char*   MNAME[3]       = { "X", "Y", "Z" };

#define BT        Serial1
#define BT_BAUD   9600
#define USB_BAUD  115200

// ==================================================================
//  DRIVE (L298N) PIN MAP
// ==================================================================
const uint8_t L_ENA = 2;
const uint8_t L_ENB = 3;
const uint8_t L_IN1 = 25;
const uint8_t L_IN2 = 23;
const uint8_t L_IN3 = 17;
const uint8_t L_IN4 = 16;

const uint8_t R_ENA = 6;
const uint8_t R_ENB = 11;
const uint8_t R_IN1 = 32;
const uint8_t R_IN2 = 47;
const uint8_t R_IN3 = 45;
const uint8_t R_IN4 = 43;

const int MIN_SPEED       = 90;
const int MAX_SPEED       = 255;
const int START_SPEED     = 100;
const int SPEED_STEP      = 25;
const int MIN_MOVE_SPEED  = 90;

int  currentSpeed  = START_SPEED;
char currentCommand = '0';               // '0' = stopped

const unsigned long FAILSAFE_TIMEOUT_MS = 5000;   // 0 disables
unsigned long lastCommandMillis = 0;

// ==================================================================
//  GRIPPER
// ==================================================================
Servo gripper;
const uint8_t SERVO_PIN = 4;

int MIN_ANGLE   = 120;
int MAX_ANGLE   = 180;
int OPEN_ANGLE  = 180;
int CLOSE_ANGLE = 130;

int  gripAngle   = 150;                  // safe centered starting point
const int GRIP_STEP        = 5;          // degrees per GP / GM nudge
const int GRIP_STEP_DELAY  = 10;         // ms per 1-degree step -> speed

int  gripDwellMs = 700;                  // settle time after open/close
bool gripAutoDetach = true;              // release torque when idle
const unsigned long GRIP_IDLE_MS = 600;  // detach this long after a move
bool          gripAttached  = false;
unsigned long gripLastMoveMs = 0;

// ==================================================================
//  ROBOT GEOMETRY  (exact from manufacturer robotGeometry.cpp)
// ==================================================================
const float GEO_R = 83.7f;    // base triangle circumradius, mm
const float GEO_r = 30.4f;    // end-effector circumradius, mm
const float GEO_L = 150.0f;   // upper arm length, mm
const float GEO_l = 280.0f;   // rod (lower arm) length, mm

// Physical home: end-effector at natural top rest, arms NOT horizontal
const float HOME_X =    0.0f;
const float HOME_Y =    0.0f;
const float HOME_Z = -130.0f;

// ------------------------------------------------------------------
// KEY POSITIONS
// ------------------------------------------------------------------
// Reachable radius collapses toward the top of the Z range. Measured
// from the IK (TMIN=-45, TMAX=85), in corrected millimetres:
//
//    Z-130    8.5 mm   <-- home: lateral reach is essentially zero
//    Z-140   25.5 mm
//    Z-150   46.0 mm
//    Z-160   72.0 mm
//    Z-170  112.5 mm
//    Z-180 and below: geometry allows ~160 mm, capped to softRmax 120
//
// This is why a bare "X50" sent while sitting at home is rejected: at
// Z-130 it is genuinely unreachable, not a firmware fault. Drop to
// Z-180 or lower before any sideways move.
//
// Bin: measured to start at X100, rim at Z-230. Releasing at X110
// puts the gripper well inside it; Z-220 is 10 mm clear of the rim,
// which is legal because keepZ forbids going BELOW -230 out there.
// Bin: measured to start at X100, rim at Z-230. binZ is deliberately
// equal to transitZ, so the weed is released from the travel height
// with no descent into the bin and no lift back out -- the drop does
// the work. Keep binZ >= keepZ (-230) or the keep-out will refuse it.
float binX  = 120.0f, binY  =   0.0f, binZ  = -200.0f;
float workX =   0.0f, workY =   0.0f, workZ = -130.0f;   // = home

// All XY travel during a PICK happens at this Z.
//
// It sits ABOVE the bin keep-out plane (keepZ = -230) on purpose.
// Because every sideways leg is flown at -200, no XY move can enter
// the bin volume whatever the weed coordinates are. The only motion
// below -230 is the vertical descent onto the weed, and that column
// is validated as outside the bin first. No staging waypoint needed.
//
// Verified by simulating all seven legs of the cycle for every legal
// weed position on a 5 mm grid (1710 positions): zero violations.
float transitZ = -180.0f;

// Depth used when a PICK command arrives with no Z token.
// CONFIRMED on the machine: -300 is both the lowest reachable point
// and the best gripping depth, and it does not touch the floor.
// This sits exactly on softZmin, which is why deltaIK carries a
// 0.01 mm tolerance on the cylinder test.
float pickZ = -300.0f;

// Safe radius for the Pi to filter detections against. Slightly
// inside softRmax so detections never sit exactly on the limit.
const float SAFE_R = 115.0f;

// Workspace cylinder (robot frame, Z negative downward)
float softZmin = -300.0f;   // MEASURED lowest reachable point
float softZmax = -130.0f;
float softRmax =  120.0f;

// Arm angle limits (degrees, positive = arm pointing downward)
float thetaMin = -45.0f;
float thetaMax =  85.0f;

// ------------------------------------------------------------------
// MECHANICAL XY LIMIT
// ------------------------------------------------------------------
// Measured with the corrected gear ratio: total Y travel is 250 mm,
// i.e. +-125 mm from centre, and X behaves the same. softRmax is set
// to 120 to keep 5 mm off the hard stop.
//
// The IK geometry itself would allow ~160 mm from Z-180 downward, so
// this cap -- not the kinematics -- is what bounds the workspace.
//
// A single radius is the right model now. The earlier per-axis-plus-
// diagonal box (xyMax/xySum) and the arm-splay limit before it were
// both artefacts of the wrong gear ratio: they were fitted to numbers
// that were themselves ~33% off. With GR corrected, the old diagonal
// measurement works out to ~123 mm radius, which the 120 mm circle
// covers. Both models are gone; softRmax (RMAX<n>) is the one knob.

// ------------------------------------------------------------------
// BIN KEEP-OUT VOLUME
// ------------------------------------------------------------------
// The bin occupies everything from keepX outward, with its rim at
// keepZ. Any point with X >= keepX and Z < keepZ is refused outright,
// including intermediate points along a path, so a move can never clip
// the bin diagonally.
//
// Applied at every Y, with no Y qualification. If the bin is actually
// narrower in Y, that costs you weeds it could otherwise reach -- it
// can be given a Y extent if that turns out to matter.
//
// keepX was 100. It is now 60, measured on the machine.
//
// CONSEQUENCE, and it is a big one: a PICK descends to pickZ (-300),
// which is below keepZ, so the descent column is inside the keep-out
// for ANY weed at X >= keepX. With keepX = 60 that means no weed at
// X >= 60 can be picked at all -- deltaIK returns IK_BINZONE and
// doPick answers ERR TARGET. The pickable strip is now X < 60, and
// the Pi filters detections on that same number (it reads kpx back
// from STATUS) so it never targets a weed the arm must refuse.
float keepX =   60.0f;   // guard starts at this X   (KPX<n>)
float keepZ = -230.0f;   // ... below this Z         (KPZ<n>)

// ==================================================================
//  MACHINE CONFIG
// ==================================================================
const long MOTOR_STEPS_REV = 200;
int   microstep  = 32;

// Gear ratio applied at boot.
//
// MEASURED 4.5, confirmed three independent ways:
//   * commanded Y span 140 mm  ->  measured 140 mm
//   * home Z-130 to lowest Z-300  ->  matches physically
//   * one full motor revolution turns the arm ~80 deg (360/80 = 4.5)
//
// The earlier value of 6.0 made every commanded coordinate read about
// 33% short of reality (commanded 75 mm actually travelled ~110 mm),
// in Z as well as XY. All distances below are now honest millimetres.
//   stepsPerRad = 200 * 32 * 4.5 / 2pi = 4583.66
const float BOOT_GEAR_RATIO = 4.5f;

float gearRatio  = 4.5f;
float stepsPerRad = 0.0f;          // computed in recalcSteps()

// arm k -> RAMPS motor index (changed by MAP command)
uint8_t mapSel = 0;
const uint8_t MAPS[6][3] = {
  {0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}
};

bool invert[3] = {true, true, true};

// ==================================================================
//  MOTION CONFIG
// ==================================================================
float feedRate   = 150.0f;   // mm/s
float travelFeed = 200.0f;
float minFeed    = 100.0f;
float accelMM    = 600.0f;   // mm/s^2
float segMM      =   2.0f;
const float MIN_STEPS_PER_SEG = 8.0f;
const float PATH_ACC_SEG      = 2.0f;

const unsigned long MIN_STEP_US = 50;  // 20 kHz hard ceiling
const uint8_t       PULSE_US    =  3;  // DRV8825 min 1.9 us

// ==================================================================
//  STATE
// ==================================================================
float curX, curY, curZ;
long  stepPos[3] = {0, 0, 0};
bool  homed      = false;
bool  driversOn  = false;
bool  relMode    = false;
volatile bool abortFlag = false;
long  jogSteps   = 200;

bool  autoPick   = false;    // AP1 -> bare X/Y/Z lines trigger a PICK
bool  pickBusy   = false;

// ==================================================================
//  HELPERS
// ==================================================================
template<typename T> void sp(T v){ Serial.print(v);   BT.print(v); }
template<typename T> void sp(T v, int dec){ Serial.print(v,dec);   BT.print(v,dec); }
template<typename T> void sl(T v){ Serial.println(v); BT.println(v); }
template<typename T> void sl(T v, int dec){ Serial.println(v,dec); BT.println(v,dec); }
void nl(){ Serial.println(); BT.println(); }

// Forward declarations (the parser is defined after these use sites)
bool moveTo(float tx, float ty, float tz, float feed);
bool moveToEx(float tx, float ty, float tz, float feed, bool cs, bool ce);
void gripperTo(int target);
void stopAll();
void printHelp();
void printPos();

float fastAtan(float x){
  float ax = fabsf(x);
  bool inv = (ax > 1.0f);
  if(inv) ax = 1.0f/ax;
  float z = ax*ax;
  float r = ax*(0.99997726f + z*(-0.33262347f + z*(0.19354346f
            + z*(-0.11643287f + z*(0.05265332f + z*(-0.01172120f))))));
  if(inv) r = 1.5707963268f - r;
  return (x < 0.0f) ? -r : r;
}

// NOTE: segMM is auto-tuned from step resolution. With the correct
// GR=6 the arm resolves ~40.7 steps/mm, so floorSeg (~0.2mm) is far
// below PATH_ACC_SEG and segMM settles at 2.0mm. Under the old wrong
// GR=1/6 it was forced to ~7mm. Moves therefore contain ~3.5x more
// segments now: smoother path following, more IK work per move.
// If motion ever sounds rough, SEG<n> is the knob to try first.
void recalcSteps(){
  stepsPerRad = (float)MOTOR_STEPS_REV * (float)microstep * gearRatio
                / (2.0f * (float)M_PI);
  float stepsPerMM = stepsPerRad / GEO_L;
  segMM = PATH_ACC_SEG;
  if(stepsPerMM > 0.001f){
    float floorSeg = MIN_STEPS_PER_SEG / stepsPerMM;
    if(floorSeg > segMM) segMM = floorSeg;
  }
}

void setDrivers(bool on){
  driversOn = on;
  for(uint8_t i = 0; i < 3; i++)
    digitalWrite(EN_PIN[i], on ? LOW : HIGH);
}

// ==================================================================
//  INVERSE KINEMATICS  (unchanged)
// ==================================================================
const char* ikMsg(IkErr e){
  switch(e){
    case IK_OK:      return "ok";
    case IK_UNREACH: return "UNREACHABLE (geometry)";
    case IK_ANG_LO:  return "ARM TOO HIGH (above TMIN)";
    case IK_ANG_HI:  return "ARM TOO LOW  (below TMAX)";
    case IK_CYL:     return "OUTSIDE WORKSPACE CYLINDER";
    case IK_BINZONE: return "WOULD HIT THE BIN";
  }
  return "?";
}

IkErr deltaIK(float x, float y, float z, float th[3], uint8_t *bad){
  if(bad) *bad = 255;

  // 0.01 mm of slack: a target sitting exactly ON a limit (pickZ at
  // softZmin, binX at softRmax) would otherwise be rejected by float
  // rounding in the path interpolation, mid-move.
  if(z < softZmin - 0.01f || z > softZmax + 0.01f ||
     sqrtf(x*x+y*y) > softRmax + 0.01f)
    return IK_CYL;

  // Bin guard, checked before any trig. Applied to every interpolated
  // point of a path, not just endpoints, so a move can never cut a
  // corner through the bin.
  if(x >= keepX && z < keepZ) return IK_BINZONE;

  float Rr = GEO_R - GEO_r;
  float s3 = sqrtf(3.0f);
  float xx = x*x, yy = y*y, zz = z*z;
  float L2 = GEO_L*GEO_L, l2 = GEO_l*GEO_l;
  float loR = thetaMin * DEG_TO_RAD;
  float hiR = thetaMax * DEG_TO_RAD;

  // ---- arm 0 : "high" , base angle 0 deg ----
  {
    float base = -(xx+yy+zz) - 2.0f*y*Rr - Rr*Rr - L2 + l2;
    float K = base/(2.0f*GEO_L) + z;
    float M = -2.0f*(Rr + y);
    float N = base/(2.0f*GEO_L) - z;
    float disc = M*M - 4.0f*K*N;
    if(disc < 0.0f || fabsf(K) < 1e-9f){ if(bad)*bad=0; return IK_UNREACH; }
    float ang = (float)M_PI/2.0f - 2.0f*fastAtan((-M - sqrtf(disc))/(2.0f*K));
    if(isnan(ang)){ if(bad)*bad=0; return IK_UNREACH; }
    if(ang < loR){ if(bad)*bad=0; return IK_ANG_LO; }
    if(ang > hiR){ if(bad)*bad=0; return IK_ANG_HI; }
    th[0] = ang;
  }

  // ---- arm 1 : "rot" , base angle 120 deg ----
  {
    float base = -(xx+yy+zz) + (y + s3*x)*Rr - Rr*Rr - L2 + l2;
    float K = base/GEO_L + 2.0f*z;
    float M = -2.0f*(2.0f*Rr - y - s3*x);
    float N = base/GEO_L - 2.0f*z;
    float disc = M*M - 4.0f*K*N;
    if(disc < 0.0f || fabsf(K) < 1e-9f){ if(bad)*bad=1; return IK_UNREACH; }
    float ang = (float)M_PI/2.0f - 2.0f*fastAtan((-M - sqrtf(disc))/(2.0f*K));
    if(isnan(ang)){ if(bad)*bad=1; return IK_UNREACH; }
    if(ang < loR){ if(bad)*bad=1; return IK_ANG_LO; }
    if(ang > hiR){ if(bad)*bad=1; return IK_ANG_HI; }
    th[1] = ang;
  }

  // ---- arm 2 : "low" , base angle 240 deg ----
  {
    float base = -(xx+yy+zz) - (-y + s3*x)*Rr - Rr*Rr - L2 + l2;
    float K = base/GEO_L + 2.0f*z;
    float M = -2.0f*(2.0f*Rr - y + s3*x);
    float N = base/GEO_L - 2.0f*z;
    float disc = M*M - 4.0f*K*N;
    if(disc < 0.0f || fabsf(K) < 1e-9f){ if(bad)*bad=2; return IK_UNREACH; }
    float ang = (float)M_PI/2.0f - 2.0f*fastAtan((-M - sqrtf(disc))/(2.0f*K));
    if(isnan(ang)){ if(bad)*bad=2; return IK_UNREACH; }
    if(ang < loR){ if(bad)*bad=2; return IK_ANG_LO; }
    if(ang > hiR){ if(bad)*bad=2; return IK_ANG_HI; }
    th[2] = ang;
  }

  return IK_OK;
}

// ==================================================================
//  STEPPING
// ==================================================================
inline void pulseMask(uint8_t fm, uint8_t lm){
  if(fm) PORTF |= fm;
  if(lm) PORTL |= lm;
  delayMicroseconds(PULSE_US);
  if(fm) PORTF &= ~fm;
  if(lm) PORTL &= ~lm;
}

// '0' aborts only when it is the FIRST character of a line, so a
// coordinate like "Z-300" arriving mid-move cannot abort that move.
// 'k'/'K' abort unconditionally.
bool abortLineStart = true;

// Bytes that arrive while a NON-PICK move is running used to be READ
// AND THROWN AWAY here. They are now parked in this buffer and
// replayed by loop() once the move finishes, which is what lets the
// Pi queue its next detection during the return-home leg after a
// pick, instead of that detection being lost.
//
// This parking is deliberately gated on pickBusy. While pickBusy is
// true -- from the moment a PICK is accepted until the gripper opens
// over the bin -- incoming bytes are discarded exactly as before, NOT
// queued. A PICK sent during that window must not be accepted for
// later execution; it must simply not be accepted. Only once the
// weed has been dropped (pickBusy goes false, right before DONE) does
// parking resume, covering just the return-to-work leg.
char    pendBuf[96];
uint8_t pendLen = 0;

inline void pollAbortChar(char c){
  if(c == '\n' || c == '\r'){
    abortLineStart = true;
  } else {
    if(c == 'k' || c == 'K' || (c == '0' && abortLineStart)) abortFlag = true;
    abortLineStart = false;
  }
  // Only queue for later execution once the current pick is no longer
  // busy (see note above). While busy, the byte is consumed here and
  // gone -- same as the old discard-only behaviour.
  if(!pickBusy && pendLen < (uint8_t)sizeof(pendBuf)) pendBuf[pendLen++] = c;
}

bool pollAbort(){
  while(BT.available())     pollAbortChar(BT.read());
  while(Serial.available()) pollAbortChar(Serial.read());
  return abortFlag;
}


int8_t lastDir[3] = {0, 0, 0};
unsigned long stepClock = 0;

void resetStepClock(){
  stepClock = 0;
  lastDir[0] = lastDir[1] = lastDir[2] = 0;
}

bool stepSegment(const long tgt[3], float segT){
  long d[3]; long mx = 0;
  bool dirPos[3];
  for(uint8_t m = 0; m < 3; m++){
    d[m] = tgt[m] - stepPos[m];
    dirPos[m] = (d[m] >= 0);
    long ad = labs(d[m]);
    if(ad > mx) mx = ad;
  }

  if(mx == 0){
    if(stepClock == 0) stepClock = micros();
    stepClock += (unsigned long)(segT * 1e6f);
    while((long)(micros() - stepClock) < 0) if(pollAbort()) return false;
    return true;
  }

  bool dirChanged = false;
  for(uint8_t m = 0; m < 3; m++){
    int8_t want = dirPos[m] ? 1 : -1;
    if(d[m] != 0 && lastDir[m] != want){
      digitalWrite(DIR_PIN[m], (dirPos[m]^invert[m]) ? HIGH : LOW);
      lastDir[m] = want;
      dirChanged = true;
    }
  }
  if(dirChanged) delayMicroseconds(3);

  long err[3] = {mx/2, mx/2, mx/2};
  unsigned long ivl = max((unsigned long)(segT*1e6f/(float)mx), MIN_STEP_US);

  unsigned long now = micros();
  if(stepClock == 0 || (long)(now - stepClock) > 20000L) stepClock = now;

  for(long i = 0; i < mx; i++){
    stepClock += ivl;
    if((long)(micros() - stepClock) > (long)ivl){
      stepClock = micros();
      if(pollAbort()) return false;
    } else {
      while((long)(micros() - stepClock) < 0) if(pollAbort()) return false;
    }
    uint8_t fm=0, lm=0;
    for(uint8_t m = 0; m < 3; m++){
      err[m] -= labs(d[m]);
      if(err[m] < 0){
        err[m] += mx;
        fm |= STEP_F_MASK[m];
        lm |= STEP_L_MASK[m];
        stepPos[m] += dirPos[m] ? 1 : -1;
      }
    }
    pulseMask(fm, lm);
  }
  return true;
}

// ==================================================================
//  CARTESIAN MOVE  (unchanged)
// ==================================================================
void reportErr(IkErr e, uint8_t arm, float x, float y, float z){
  sp(F("!! REJECTED X")); sp(x); sp(F(" Y")); sp(y); sp(F(" Z")); sp(z);
  sp(F("  -> ")); sp(ikMsg(e));
  if(arm < 3){ sp(F(" [arm ")); sp(arm); sp(F("]")); }
  nl();
}

bool moveToEx(float tx, float ty, float tz, float feed,
              bool contStart, bool contEnd){
  if(!homed){ sl(F("!! SETHOME first")); return false; }

  float dx=tx-curX, dy=ty-curY, dz=tz-curZ;
  float dist = sqrtf(dx*dx+dy*dy+dz*dz);
  if(dist < 0.01f) return true;

  float th[3]; uint8_t bad;
  IkErr e = deltaIK(tx,ty,tz,th,&bad);
  if(e != IK_OK){ reportErr(e,bad,tx,ty,tz); return false; }

  int nseg = max(1, (int)ceilf(dist/segMM));
  if(nseg > 3000) nseg = 3000;
  int vstep = max(1, nseg/24);
  for(int i=1; i<=nseg; i+=vstep){
    float f=i/(float)nseg;
    e = deltaIK(curX+dx*f, curY+dy*f, curZ+dz*f, th, &bad);
    if(e != IK_OK){
      sl(F("!! PATH exits workspace, cancelled"));
      reportErr(e,bad, curX+dx*f, curY+dy*f, curZ+dz*f);
      return false;
    }
  }

  if(!driversOn) setDrivers(true);
  abortFlag = false;
  abortLineStart = true;               // fresh line context for the poller
  if(!contStart) resetStepClock();
  float segLen  = dist / (float)nseg;
  float minF2   = minFeed * minFeed;
  float twoA    = 2.0f * accelMM;

  float sx=curX, sy=curY, sz=curZ;
  for(int i = 1; i <= nseg; i++){
    float f   = i / (float)nseg;
    float px  = curX+dx*f, py = curY+dy*f, pz = curZ+dz*f;
    // The pre-flight check above only samples ~24 points, so a narrow
    // violation can slip through. If IK fails HERE, th[] still holds
    // angles from the previous segment (or a half-written set), and
    // using them would command the arms to a wildly wrong pose. Bail
    // out cleanly instead.
    if(deltaIK(px, py, pz, th, &bad) != IK_OK){
      curX=sx; curY=sy; curZ=sz;
      sl(F("!! IK failed mid-path, stopped"));
      reportErr(IK_CYL, bad, px, py, pz);
      return false;
    }

    long tgt[3];
    for(uint8_t k = 0; k < 3; k++){
      uint8_t m = MAPS[mapSel][k];
      tgt[m] = lroundf(th[k] * stepsPerRad);
    }

    float sFront = segLen * (i - 1);
    float sBack  = dist - segLen * i;
    float vA = contStart ? feed : sqrtf(minF2 + twoA * max(sFront, 0.0f));
    float vD = contEnd   ? feed : sqrtf(minF2 + twoA * max(sBack,  0.0f));
    float v  = max(min(feed, min(vA, vD)), minFeed);

    if(!stepSegment(tgt, segLen / v)){
      curX=sx; curY=sy; curZ=sz;
      sl(F("** ABORTED"));
      abortFlag=false;
      return false;
    }
    sx=px; sy=py; sz=pz;
  }
  curX=tx; curY=ty; curZ=tz;
  return true;
}

bool moveTo(float tx, float ty, float tz, float feed){
  return moveToEx(tx, ty, tz, feed, false, false);
}

// ==================================================================
//  GRIPPER
// ==================================================================
void gripAttach(){
  if(!gripAttached){
    gripper.attach(SERVO_PIN);
    gripper.write(gripAngle);
    gripAttached = true;
  }
  gripLastMoveMs = millis();
}

void gripDetach(){
  if(gripAttached){
    gripper.detach();
    gripAttached = false;
  }
}

// Auto-release holding torque a short time after the last move.
// This kills MG995 jitter/heat AND stops the Servo Timer5 ISR, which
// removes its (small) timing jitter from the stepper pulse train.
void gripperIdleTask(){
  if(gripAutoDetach && gripAttached && !pickBusy &&
     (millis() - gripLastMoveMs) > GRIP_IDLE_MS){
    gripDetach();
  }
}

void reportAngle(){
  sp(F("Grip angle: ")); sl(gripAngle);
}

// Moves one degree at a time so speed = GRIP_STEP_DELAY and a bad
// target never yanks the gripper into a hard stop.
void gripperTo(int target){
  target = constrain(target, MIN_ANGLE, MAX_ANGLE);
  gripAttach();
  int step = (target > gripAngle) ? 1 : -1;
  while(gripAngle != target){
    gripAngle += step;
    gripper.write(gripAngle);
    delay(GRIP_STEP_DELAY);
  }
  gripLastMoveMs = millis();
  reportAngle();
}

void gripOpen(){  gripperTo(OPEN_ANGLE);  delay(gripDwellMs); }
void gripClose(){ gripperTo(CLOSE_ANGLE); delay(gripDwellMs); }

void gripSweep(){
  sl(F("Sweeping MIN -> MAX -> center. Listen for binding."));
  gripperTo(MIN_ANGLE);
  delay(500);
  gripperTo(MAX_ANGLE);
  delay(500);
  gripperTo((MIN_ANGLE + MAX_ANGLE) / 2);
}

// ==================================================================
//  COMBINED POWER-DOWN  ('D' / 'E')
// ==================================================================
// D releases EVERYTHING the arm holds: steppers unpowered, servo
// detached, so the whole mechanism can be moved by hand. Because the
// arms can now be moved without the firmware knowing, stepPos[] and
// curX/Y/Z become fiction -- so D also clears the homed flag. Without
// that, the next coordinate command would plan a move from a starting
// point that no longer exists and slam the arms toward it at full feed.
//
// Use DS when you only want the DRV8825s to stop heating and you know
// nothing has been moved: that keeps the home reference intact.
void doDisableAll(){
  setDrivers(false);
  gripDetach();
  homed = false;
  sl(F("ALL MOTORS RELEASED (steppers off, servo detached)"));
  sl(F("home reference cleared -- send SETHOME after repositioning"));
}

void doEnableAll(){
  setDrivers(true);
  // The servo jumps to gripAngle the instant it is re-attached. If the
  // jaws were moved by hand while detached, that is a sudden snap --
  // keep fingers clear.
  gripAttach();
  sl(F("steppers ON, servo attached (jaws may snap to last angle)"));
  if(!homed) sl(F("not homed -- place delta at TOP rest, then SETHOME"));
}

// ==================================================================
//  DRIVE MOTORS
// ==================================================================
int effectiveSpeed(){
  if(currentSpeed > 0 && currentSpeed < MIN_MOVE_SPEED) return MIN_MOVE_SPEED;
  return currentSpeed;
}

// dir: 1 = forward, -1 = backward, 0 = stop
void setMotorDirection(uint8_t inPinA, uint8_t inPinB, int dir){
  if(dir == 1){        digitalWrite(inPinA, HIGH); digitalWrite(inPinB, LOW);  }
  else if(dir == -1){  digitalWrite(inPinA, LOW);  digitalWrite(inPinB, HIGH); }
  else {               digitalWrite(inPinA, LOW);  digitalWrite(inPinB, LOW);  }
}

void leftMotors(int dir){
  setMotorDirection(L_IN1, L_IN2, dir);
  setMotorDirection(L_IN3, L_IN4, dir);
  int pwm = (dir == 0) ? 0 : effectiveSpeed();
  analogWrite(L_ENA, pwm);
  analogWrite(L_ENB, pwm);
  Serial.print(F("  [LEFT ] dir=")); Serial.print(dir);
  Serial.print(F(" pwm="));          Serial.println(pwm);
}

void rightMotors(int dir){
  setMotorDirection(R_IN1, R_IN2, dir);
  setMotorDirection(R_IN3, R_IN4, dir);
  int pwm = (dir == 0) ? 0 : effectiveSpeed();
  analogWrite(R_ENA, pwm);
  analogWrite(R_ENB, pwm);
  Serial.print(F("  [RIGHT] dir=")); Serial.print(dir);
  Serial.print(F(" pwm="));          Serial.println(pwm);
}

void stopAll(){
  leftMotors(0);
  rightMotors(0);
}

void applyMovement(char cmd){
  switch(cmd){
    case 'F':
      Serial.println(F("[MOVE] FORWARD - left:FWD right:FWD"));
      leftMotors(1);  rightMotors(1);  break;
    case 'B':
      Serial.println(F("[MOVE] BACKWARD - left:BACK right:BACK"));
      leftMotors(-1); rightMotors(-1); break;
    case 'R':
      Serial.println(F("[MOVE] PIVOT RIGHT - left:FWD right:BACK"));
      leftMotors(1);  rightMotors(-1); break;
    case 'L':
      Serial.println(F("[MOVE] PIVOT LEFT - left:BACK right:FWD"));
      leftMotors(-1); rightMotors(1);  break;
    case '0':
    default:
      Serial.println(F("[MOVE] STOP - all motors off"));
      stopAll(); break;
  }
}

void driveCommand(char cmd){
  switch(cmd){
    case 'F': case 'B': case 'R': case 'L': case '0':
      lastCommandMillis = millis();
      currentCommand = cmd;
      Serial.print(F("[CMD] Movement command: ")); Serial.println(cmd);
      applyMovement(currentCommand);
      break;
    case 'T':
      currentSpeed += SPEED_STEP;
      if(currentSpeed > MAX_SPEED) currentSpeed = MAX_SPEED;
      Serial.print(F("[SPEED] Increased -> ")); Serial.println(currentSpeed);
      applyMovement(currentCommand);
      break;
    case 'X':
      currentSpeed -= SPEED_STEP;
      if(currentSpeed < MIN_SPEED) currentSpeed = MIN_SPEED;
      Serial.print(F("[SPEED] Decreased -> ")); Serial.println(currentSpeed);
      applyMovement(currentCommand);
      break;
    default: break;
  }
}

void driveWatchdog(){
  if(FAILSAFE_TIMEOUT_MS > 0 && currentCommand != '0'){
    if(millis() - lastCommandMillis > FAILSAFE_TIMEOUT_MS){
      Serial.println(F("[WATCHDOG] No command in time -> auto-stopping"));
      currentCommand = '0';
      applyMovement(currentCommand);
    }
  }
}

// ==================================================================
//  AUTONOMOUS PICK CYCLE  (the new feature)
// ==================================================================
void pickFail(const __FlashStringHelper *why){
  pickBusy = false;
  sp(F("ERR ")); sl(why);
}

void doPick(float x, float y, float z){
  if(pickBusy){ sl(F("ERR BUSY")); return; }
  if(!homed)  { sl(F("ERR NOTHOMED")); return; }

  // Validate EVERY waypoint before touching the hardware, so we never
  // grab a weed and then discover we cannot get to the bin.
  float th[3]; uint8_t bad;
  if(deltaIK(x, y, z, th, &bad) != IK_OK)              { sl(F("ERR TARGET"));  return; }
  if(deltaIK(x, y, transitZ, th, &bad) != IK_OK)       { sl(F("ERR TRANSIT")); return; }
  if(deltaIK(binX, binY, binZ, th, &bad) != IK_OK)     { sl(F("ERR BIN"));     return; }
  if(deltaIK(workX, workY, workZ, th, &bad) != IK_OK)  { sl(F("ERR WORKPOS")); return; }

  pickBusy = true;

  // Safety: never drive the base while the arm is out.
  currentCommand = '0';
  stopAll();

  sp(F("-- PICK X")); sp(x); sp(F(" Y")); sp(y); sp(F(" Z")); sl(z);

  // Sideways legs are flown at transitZ, which is above the bin
  // keep-out plane, so no XY move can clip the bin. Only the descent
  // onto the weed goes below it, and that column is validated above.
  gripOpen();                                                                     // 1
  if(!moveTo(x, y, transitZ, travelFeed))       { pickFail(F("MOVE_TRANSIT")); return; }  // 2
  if(!moveTo(x, y, z, feedRate * 0.5f))         { pickFail(F("MOVE_DOWN"));    return; }  // 3
  gripClose();                                                                    // 4
  if(!moveTo(x, y, transitZ, feedRate * 0.5f))  { pickFail(F("MOVE_LIFT"));    return; }  // 5

  // binZ is normally the same as transitZ, so this is a flat XY move
  // straight across to the bin -- no descent, no lift back out.
  if(!moveTo(binX, binY, binZ, travelFeed))     { pickFail(F("MOVE_BIN"));     return; }  // 6
  gripOpen();                                                                     // 7

  // DONE goes out HERE, the instant the weed is released -- not after
  // the arm has finished travelling home. The Pi can send the next
  // detection immediately; bytes arriving during the return leg are
  // parked by pollAbort() and replayed by loop() when it completes.
  pickBusy = false;
  sl(F("DONE"));

  // Return to the working position. Best-effort: the cycle already
  // succeeded, so a failure here is reported as a warning and must
  // NOT turn into an ERR the Pi would read as a failed pick.
  if(!moveTo(workX, workY, workZ, travelFeed))
    sl(F("!! could not return to work position"));
}

// ==================================================================
//  HIGH LEVEL
// ==================================================================
void doSetHome(){
  float th[3]; uint8_t bad;
  IkErr e = deltaIK(HOME_X, HOME_Y, HOME_Z, th, &bad);
  if(e != IK_OK){
    sl(F("!! home position outside workspace - check TMIN/TMAX"));
    return;
  }
  for(uint8_t k=0; k<3; k++){
    uint8_t m = MAPS[mapSel][k];
    stepPos[m] = lroundf(th[k] * stepsPerRad);
  }
  curX=HOME_X; curY=HOME_Y; curZ=HOME_Z;
  homed=true;
  sp(F("Home set: X")); sp(HOME_X); sp(F(" Y")); sp(HOME_Y);
  sp(F(" Z")); sl(HOME_Z);
  sp(F("Arm angles: "));
  for(uint8_t k=0;k<3;k++){ sp(th[k]*RAD_TO_DEG); sp(F("  ")); } nl();
  sp(F("Step offsets: "));
  for(uint8_t m=0;m<3;m++){ sp(stepPos[m]); sp(F("  ")); } nl();
}

void doReach(float x, float y, float z){
  float th[3]; uint8_t bad;
  IkErr e = deltaIK(x,y,z,th,&bad);
  if(e!=IK_OK){ reportErr(e,bad,x,y,z); return; }
  sp(F("REACHABLE X")); sp(x); sp(F(" Y")); sp(y); sp(F(" Z")); sp(z);
  sp(F("  arms(deg): "));
  for(uint8_t k=0;k<3;k++){ sp(th[k]*RAD_TO_DEG,2); sp(F("  ")); }
  nl();
}

// WEED is the pick sequence without the bin trip. Like PICK it flies
// sideways at transitZ, otherwise it fails the same way PICK did.
void doWeed(float x, float y, float z){
  sl(F("-- weed --"));
  gripOpen();
  if(!moveTo(x,y,transitZ, travelFeed)) return;
  if(!moveTo(x,y,z,        feedRate*0.5f)) return;
  gripClose();
  if(!moveTo(x,y,transitZ, feedRate*0.5f)) return;
  sl(F("  plucked"));
}

// ------------------------------------------------------------------
// ONE-SHOT RELATIVE CARTESIAN JOG   JOGX<n> / JOGY<n> / JOGZ<n>
// ------------------------------------------------------------------
// The web panel needs "move 10 mm in +X from wherever you are". The
// existing way to do that is REL, then X10, then ABS -- three lines,
// and if anything goes wrong in between the machine is left in REL and
// the NEXT absolute coordinate is silently treated as an offset. That
// is a genuinely dangerous failure mode with a remote client, so this
// does the whole thing in one command and never touches relMode.
//
// Refused unless homed (the move would start from a fictional position)
// and refused while a PICK is running. Always answers with exactly one
// of "OK JOG ..." or "ERR ...", so the Pi can gate on the reply the
// same way it gates on DONE.
void doJogAxis(uint8_t axis, float mm){
  if(pickBusy){ sl(F("ERR BUSY"));     return; }
  if(!homed)  { sl(F("ERR NOTHOMED")); return; }

  float tx = curX, ty = curY, tz = curZ;
  if     (axis == 0) tx += mm;
  else if(axis == 1) ty += mm;
  else               tz += mm;

  if(!moveTo(tx, ty, tz, feedRate)){ sl(F("ERR JOG")); return; }

  sp(F("OK JOG X")); sp(curX,1);
  sp(F(" Y"));       sp(curY,1);
  sp(F(" Z"));       sl(curZ,1);
}

void doCircle(){
  float cz = (softZmin+softZmax)*0.5f;
  float rr = min(50.0f, softRmax*0.4f);
  if(!moveTo(rr,0,cz,travelFeed)) return;
  for(int a=10;a<=360;a+=5){
    float t=a*DEG_TO_RAD;
    bool first = (a==10), last = (a>=360);
    if(!moveToEx(rr*cosf(t), rr*sinf(t), cz, feedRate, !first, !last)) return;
  }
  sl(F("circle done"));
}

void doSquare(){
  float cz = (softZmin+softZmax)*0.5f;
  float s = min(40.0f, softRmax*0.35f);
  if(!moveTo(-s,-s,cz,travelFeed)) return;
  if(!moveTo( s,-s,cz,feedRate))   return;
  if(!moveTo( s, s,cz,feedRate))   return;
  if(!moveTo(-s, s,cz,feedRate))   return;
  if(!moveTo(-s,-s,cz,feedRate))   return;
  moveTo(0,0,cz,travelFeed);
  sl(F("square done"));
}

void rawMove(uint8_t motor, long steps){
  if(homed){ sl(F("!! UNHOME before raw jog")); return; }
  if(!driversOn) setDrivers(true);
  long tgt[3]={stepPos[0],stepPos[1],stepPos[2]};
  tgt[motor]+=steps;
  abortFlag=false;
  resetStepClock();
  stepSegment(tgt, fabsf((float)steps)/600.0f);
  abortFlag=false;
}

void doRawSteps(uint8_t arm, long steps){
  uint8_t m = MAPS[mapSel][arm];
  sp(F("RAW arm")); sp(arm); sp(F(" motor ")); sp(MNAME[m]);
  sp(F("  ")); sp(steps); sl(F(" steps (gearRatio NOT applied)"));
  sp(F("  = ")); sp((float)steps/(float)(MOTOR_STEPS_REV*microstep),4);
  sl(F(" motor revolutions"));
  rawMove(m, steps);
  sl(F("  measure arm rotation A (deg), then:  GR = 360 * motorRevs / A"));
}

void doCal(uint8_t arm, float deg){
  uint8_t m=MAPS[mapSel][arm];
  long st=lroundf(deg*DEG_TO_RAD*stepsPerRad);
  sp(F("CAL arm")); sp(arm); sp(F(" motor ")); sp(MNAME[m]);
  sp(F("  cmd ")); sp(deg); sp(F("deg = ")); sp(st); sl(F(" steps"));
  rawMove(m, st);
}

void doIdent(){
  sl(F("IDENT: wiggling each motor..."));
  for(uint8_t k=0;k<3;k++){
    uint8_t m=MAPS[mapSel][k];
    sp(F("  arm")); sp(k); sp(F(" = motor ")); sl(MNAME[m]);
    rawMove(m, 300); delay(300);
    rawMove(m,-300); delay(600);
  }
  sl(F("arm0 must be the -Y side arm. Use MAP0..5 to remap."));
}

// ==================================================================
//  STATUS
// ==================================================================
void printHelp(){
  sl(F("=== WEED ROBOT MASTER ==="));
  sl(F("-- drive (single char) --"));
  sl(F("F B R L   T/X speed   0 = STOP + abort"));
  sl(F("-- autonomous --"));
  sl(F("PICK X Y Z   full cycle, replies DONE"));
  sl(F("AP1/AP0  bare XYZ triggers PICK   WORK  BIN"));
  sl(F("-- delta --"));
  sl(F("X<n> Y<n> Z<n> [F<n>]   REACH X Y Z   WEED X Y Z"));
  sl(F("HOME PARK CIRCLE SQUARE  ABS|REL  POS LIMITS"));
  sl(F("JOGX<n> JOGY<n> JOGZ<n>  one-shot relative jog in mm"));
  sl(F("STATUS   one parsable ST key=value line (for the Pi web panel)"));
  sl(F("-- power --"));
  sl(F("D = release ALL (steppers+servo, clears home)   E = enable all"));
  sl(F("DS/ES steppers only (home kept)   GDET/GATT servo only"));
  sl(F("-- gripper --"));
  sl(F("GO GC  GP GM  GA<n>  GSWEEP"));
  sl(F("GOP<n> GCL<n> GMIN<n> GMAX<n> GDWL<n> GAD<n>"));
  sl(F("-- setup --"));
  sl(F("SETHOME UNHOME  GR<n> MS<n> MAP0..5 IDENT IX IY IZ"));
  sl(F("SX/SY/SZ<n>  CALX/CALY/CALZ<n>"));
  sl(F("TMIN TMAX RMAX ZMIN ZMAX  ACC<n> SEG<n>"));
  sl(F("TRZ<n> transit Z (XY travel height)   PZ<n> default weed depth"));
  sl(F("KPX<n> KPZ<n>  bin keep-out volume"));
  sl(F("BX BY BZ SETBIN   WX WY WZ SETWORK"));
  sl(F("JF JB JR JL JT JX  raw jog (unhomed only)"));
}

void printPos(){
  sp(F("homed=")); sp(homed?F("Y"):F("N"));
  sp(F("  mode=")); sp(relMode?F("REL"):F("ABS"));
  sp(F("  drv=")); sl(driversOn?F("ON"):F("OFF"));
  if(homed){
    sp(F("  pos: X")); sp(curX); sp(F(" Y")); sp(curY); sp(F(" Z")); sl(curZ);
    float th[3]; uint8_t bad;
    if(deltaIK(curX,curY,curZ,th,&bad)==IK_OK){
      sp(F("  arms deg: "));
      for(uint8_t k=0;k<3;k++){ sp(th[k]*RAD_TO_DEG,2); sp(F("  ")); } nl();
    }
  }
  sp(F("  steps: "));
  for(uint8_t m=0;m<3;m++){ sp(MNAME[m]); sp(F("=")); sp(stepPos[m]); sp(F(" ")); } nl();
  sp(F("  stepsPerRad=")); sl(stepsPerRad,4);
  sp(F("  grip=")); sp(gripAngle);
  sp(F(" attached=")); sp(gripAttached?F("Y"):F("N"));
  sp(F("  drive=")); sp(currentCommand); sp(F(" spd=")); sl(currentSpeed);
}

// ------------------------------------------------------------------
// MACHINE-READABLE STATUS  (added for demo.py's web control panel)
// ------------------------------------------------------------------
// printPos() above is for a human reading a serial monitor: several
// lines, prose, units mixed in. The Pi needs something it can parse
// once a second without guessing, so this emits ONE line of key=value
// pairs with a fixed "ST " prefix:
//
//   ST homed=1 busy=0 drv=1 att=0 grip=150 x=0.0 y=0.0 z=-130.0
//      drive=0 spd=100 ap=0 rel=0 pz=-300.0 trz=-180.0 rmax=120.0
//      kpx=60.0 kpz=-230.0
//
// demo.py filters lines starting with "ST " out of the operator console
// and folds them into the status panel instead, so polling this often
// does not drown the log. Keep it to ONE line and keep the prefix.
void printStatus(){
  sp(F("ST homed=")); sp(homed ? 1 : 0);
  sp(F(" busy="));    sp(pickBusy ? 1 : 0);
  sp(F(" drv="));     sp(driversOn ? 1 : 0);
  sp(F(" att="));     sp(gripAttached ? 1 : 0);
  sp(F(" grip="));    sp(gripAngle);
  sp(F(" x="));       sp(curX, 1);
  sp(F(" y="));       sp(curY, 1);
  sp(F(" z="));       sp(curZ, 1);
  sp(F(" drive="));   sp(currentCommand);
  sp(F(" spd="));     sp(currentSpeed);
  sp(F(" ap="));      sp(autoPick ? 1 : 0);
  sp(F(" rel="));     sp(relMode ? 1 : 0);
  sp(F(" pz="));      sp(pickZ, 1);
  sp(F(" trz="));     sp(transitZ, 1);
  sp(F(" rmax="));    sp(softRmax, 1);
  sp(F(" kpx="));     sp(keepX, 1);
  sp(F(" kpz="));     sp(keepZ, 1);
  nl();
}

void printLimits(){
  sp(F("workspace: r<=")); sp(softRmax);
  sp(F("  Z ")); sp(softZmin); sp(F(" to ")); sl(softZmax);
  sp(F("arm limits: TMIN=")); sp(thetaMin); sp(F(" TMAX=")); sl(thetaMax);
  sp(F("bin keep-out: X>=")); sp(keepX); sp(F("  Z<")); sl(keepZ);
  sp(F("safe radius for Pi: ")); sl(SAFE_R);
  sp(F("accel=")); sp(accelMM); sp(F("  seg=")); sl(segMM);
  sp(F("GR=")); sp(gearRatio,6); sp(F("  MS=1/")); sl(microstep);
  sp(F("map=")); sp(mapSel); sp(F("  invert: "));
  for(uint8_t m=0;m<3;m++){ sp(MNAME[m]); sp(invert[m]?F("=Y "):F("=N ")); } nl();
  sp(F("bin:  X")); sp(binX);  sp(F(" Y")); sp(binY);  sp(F(" Z")); sl(binZ);
  sp(F("work: X")); sp(workX); sp(F(" Y")); sp(workY); sp(F(" Z")); sl(workZ);
  sp(F("transitZ=")); sp(transitZ);
  sp(F("  pickZ=")); sl(pickZ);
  sp(F("autoPick=")); sl(autoPick?F("ON"):F("OFF"));
  sp(F("grip: open=")); sp(OPEN_ANGLE); sp(F(" close=")); sp(CLOSE_ANGLE);
  sp(F(" min=")); sp(MIN_ANGLE); sp(F(" max=")); sp(MAX_ANGLE);
  sp(F(" dwell=")); sl(gripDwellMs);
}

// ==================================================================
//  PARSER
// ==================================================================
char    lineBuf[96];
uint8_t lineLen = 0;
unsigned long lastRxMs = 0;

// Single characters sent by a phone BT app usually arrive with NO
// newline. If exactly one character has been sitting in the buffer for
// SINGLE_CHAR_FLUSH_MS, treat it as a complete command. Restricting the
// flush to length 1 means a real multi-token line can never be
// mangled by the timeout.
const unsigned long SINGLE_CHAR_FLUSH_MS = 60;

bool splitToken(const char *s, Token &t){
  t.alen=0; t.hasVal=false; t.val=0.0f;
  uint8_t i=0;
  while(s[i] && isAlpha(s[i]) && t.alen<7) t.alpha[t.alen++]=toupper(s[i++]);
  t.alpha[t.alen]=0;
  if(s[i]){
    char num[16]; uint8_t n=0;
    if(s[i]=='+'||s[i]=='-') num[n++]=s[i++];
    while(s[i]&&(isDigit(s[i])||s[i]=='.')&&n<15) num[n++]=s[i++];
    num[n]=0;
    if(n>0&&!(n==1&&(num[0]=='+'||num[0]=='-'))){ t.val=atof(num); t.hasVal=true; }
  }
  return t.alen>0||t.hasVal;
}

bool streq(const char *a, const char *b){ return strcmp(a,b)==0; }

void runLine(){
  lineBuf[lineLen]=0; lineLen=0;
  if(!lineBuf[0]) return;

  bool hasX=false,hasY=false,hasZ=false;
  float vx=curX,vy=curY,vz=curZ,vf=feedRate;
  uint8_t action=0;  // 0=move 1=reach 2=weed 3=pick

  char *tok=strtok(lineBuf," \t,");
  while(tok){
    Token t;
    if(splitToken(tok,t)){
      const char *a=t.alpha;

      // ---------- single letter WITH a value ----------
      if(t.alen==1&&t.hasVal){
        switch(a[0]){
          case 'X': vx=t.val; hasX=true; break;
          case 'Y': vy=t.val; hasY=true; break;
          case 'Z': vz=t.val; hasZ=true; break;
          case 'F': vf=t.val; break;
          case 'G': break;
          default:  break;
        }
      }
      // ---------- multi letter WITH a value ----------
      else if(t.alen>=2&&t.hasVal){
        if     (streq(a,"GR"))  { gearRatio=t.val; recalcSteps(); sp(F("GR=")); sl(gearRatio,6); }
        else if(streq(a,"MS"))  { microstep=(int)t.val; recalcSteps(); sp(F("MS=1/")); sl(microstep); }
        else if(streq(a,"TMIN")){ thetaMin=t.val; sp(F("TMIN=")); sl(thetaMin); }
        else if(streq(a,"TMAX")){ thetaMax=t.val; sp(F("TMAX=")); sl(thetaMax); }
        else if(streq(a,"RMAX")){ softRmax=t.val; sp(F("RMAX=")); sl(softRmax); }
        else if(streq(a,"ZMIN")){ softZmin=t.val; sp(F("ZMIN=")); sl(softZmin); }
        else if(streq(a,"ZMAX")){ softZmax=t.val; sp(F("ZMAX=")); sl(softZmax); }
        else if(streq(a,"ACC")) { accelMM=t.val;  sp(F("ACC=")); sl(accelMM); }
        else if(streq(a,"SEG")) { segMM=max(0.05f,t.val); sp(F("SEG=")); sl(segMM); }
        else if(streq(a,"MAP")) { if(t.val>=0&&t.val<6){mapSel=(uint8_t)t.val; sp(F("map=")); sl(mapSel);} }
        else if(streq(a,"JOG")) { jogSteps=(long)t.val; }
        // One-shot relative Cartesian jog, in mm. streq is an exact
        // match, so "JOGX" never collides with the "JOG" raw-step-size
        // setter directly above it.
        else if(streq(a,"JOGX")) doJogAxis(0, t.val);
        else if(streq(a,"JOGY")) doJogAxis(1, t.val);
        else if(streq(a,"JOGZ")) doJogAxis(2, t.val);
        // --- bin / working position ---
        else if(streq(a,"BX"))  { binX=t.val;  sp(F("binX="));  sl(binX);  }
        else if(streq(a,"BY"))  { binY=t.val;  sp(F("binY="));  sl(binY);  }
        else if(streq(a,"BZ"))  { binZ=t.val;  sp(F("binZ="));  sl(binZ);  }
        else if(streq(a,"WX"))  { workX=t.val; sp(F("workX=")); sl(workX); }
        else if(streq(a,"WY"))  { workY=t.val; sp(F("workY=")); sl(workY); }
        else if(streq(a,"WZ"))  { workZ=t.val; sp(F("workZ=")); sl(workZ); }
        else if(streq(a,"TRZ")) { transitZ=t.val; sp(F("transitZ=")); sl(transitZ); }
        else if(streq(a,"KPX"))  { keepX=t.val; sp(F("keepX=")); sl(keepX); }
        else if(streq(a,"KPZ"))  { keepZ=t.val; sp(F("keepZ=")); sl(keepZ); }
        else if(streq(a,"PZ"))  { pickZ=t.val;    sp(F("pickZ="));    sl(pickZ); }
        else if(streq(a,"AP"))  { autoPick=(t.val!=0); sp(F("autoPick=")); sl(autoPick?F("ON"):F("OFF")); }
        // --- gripper tuning ---
        else if(streq(a,"GA"))  gripperTo((int)t.val);
        else if(streq(a,"GOP")) { OPEN_ANGLE =(int)t.val; sp(F("openAng="));  sl(OPEN_ANGLE);  }
        else if(streq(a,"GCL")) { CLOSE_ANGLE=(int)t.val; sp(F("closeAng=")); sl(CLOSE_ANGLE); }
        else if(streq(a,"GMIN")){ MIN_ANGLE  =(int)t.val; sp(F("gMin=")); sl(MIN_ANGLE); }
        else if(streq(a,"GMAX")){ MAX_ANGLE  =(int)t.val; sp(F("gMax=")); sl(MAX_ANGLE); }
        else if(streq(a,"GDWL")){ gripDwellMs=(int)t.val; sp(F("gDwell=")); sl(gripDwellMs); }
        else if(streq(a,"GAD")) { gripAutoDetach=(t.val!=0); sp(F("gAutoDetach=")); sl(gripAutoDetach?F("ON"):F("OFF")); }
        // --- calibration ---
        else if(streq(a,"SX"))   doRawSteps(0,(long)t.val);
        else if(streq(a,"SY"))   doRawSteps(1,(long)t.val);
        else if(streq(a,"SZ"))   doRawSteps(2,(long)t.val);
        else if(streq(a,"CALX")) doCal(0,t.val);
        else if(streq(a,"CALY")) doCal(1,t.val);
        else if(streq(a,"CALZ")) doCal(2,t.val);
        else { sp(F("? ")); sl(a); }
      }
      // ---------- multi letter, NO value ----------
      else if(t.alen>=2){
        if     (streq(a,"HELP"))   printHelp();
        else if(streq(a,"POS"))    printPos();
        else if(streq(a,"STATUS")) printStatus();
        else if(streq(a,"LIMITS")) printLimits();
        else if(streq(a,"SETHOME"))doSetHome();
        else if(streq(a,"UNHOME")) { homed=false; sl(F("unhomed")); }
        else if(streq(a,"HOME"))   { moveTo(HOME_X,HOME_Y,HOME_Z,travelFeed); sl(F("at home")); }
        else if(streq(a,"PARK"))   moveTo(0,0,(softZmin+softZmax)*0.5f,travelFeed);
        else if(streq(a,"ABS"))    { relMode=false; sl(F("ABS")); }
        else if(streq(a,"REL"))    { relMode=true;  sl(F("REL")); }
        else if(streq(a,"REACH"))  action=1;
        else if(streq(a,"WEED"))   action=2;
        else if(streq(a,"PICK"))   action=3;
        else if(streq(a,"CIRCLE")) doCircle();
        else if(streq(a,"SQUARE")) doSquare();
        else if(streq(a,"IDENT"))  doIdent();
        else if(streq(a,"STOP"))   { abortFlag=false; driveCommand('0'); sl(F("stop")); }
        // --- bin / work positions ---
        else if(streq(a,"BIN"))    moveTo(binX,binY,binZ,travelFeed);
        else if(streq(a,"WORK"))   moveTo(workX,workY,workZ,travelFeed);
        else if(streq(a,"SETBIN")) { binX=curX;  binY=curY;  binZ=curZ;  sl(F("bin  = current pos")); }
        else if(streq(a,"SETWORK")){ workX=curX; workY=curY; workZ=curZ; sl(F("work = current pos")); }
        // --- gripper ---
        else if(streq(a,"GO"))     gripperTo(OPEN_ANGLE);
        else if(streq(a,"GC"))     gripperTo(CLOSE_ANGLE);
        else if(streq(a,"GP"))     gripperTo(gripAngle + GRIP_STEP);
        else if(streq(a,"GM"))     gripperTo(gripAngle - GRIP_STEP);
        else if(streq(a,"GSWEEP")) gripSweep();
        else if(streq(a,"GDET"))   { gripDetach(); sl(F("servo detached")); }
        else if(streq(a,"GATT"))   { gripAttach(); sl(F("servo attached")); }
        // --- steppers only: keeps the home reference intact, unlike D ---
        else if(streq(a,"DS"))     { setDrivers(false); sl(F("steppers OFF (home kept)")); }
        else if(streq(a,"ES"))     { setDrivers(true);  sl(F("steppers ON")); }
        // --- raw jog: renamed from F/B/R/L/T/X to free those letters
        //     for the drive base (see notes) ---
        else if(streq(a,"JF")) rawMove(0, jogSteps);
        else if(streq(a,"JB")) rawMove(0,-jogSteps);
        else if(streq(a,"JR")) rawMove(1, jogSteps);
        else if(streq(a,"JL")) rawMove(1,-jogSteps);
        else if(streq(a,"JT")) rawMove(2, jogSteps);
        else if(streq(a,"JX")) rawMove(2,-jogSteps);
        else if(streq(a,"IX")){ invert[0]=!invert[0]; sp(F("X inv=")); sl(invert[0]); }
        else if(streq(a,"IY")){ invert[1]=!invert[1]; sp(F("Y inv=")); sl(invert[1]); }
        else if(streq(a,"IZ")){ invert[2]=!invert[2]; sp(F("Z inv=")); sl(invert[2]); }
        else { sp(F("? ")); sl(a); }
      }
      // ---------- single letter, NO value ----------
      else if(t.alen==1&&!t.hasVal){
        switch(a[0]){
          // drive base (was raw stepper jog in the old IK sketch)
          case 'F': case 'B': case 'R': case 'L':
          case 'T': case 'X': driveCommand(a[0]); break;
          case 'E': doEnableAll();  break;
          case 'D': doDisableAll(); break;
          case 'H': printHelp(); break;
          case 'P': printPos();  break;
          default: break;
        }
      }
    }
    tok=strtok(NULL," \t,");
  }

  if(hasX||hasY||hasZ||action==3){
    float tx,ty,tz;
    if(relMode){
      tx=curX+(hasX?vx:0);
      ty=curY+(hasY?vy:0);
      tz=curZ+(hasZ?vz:0);
    } else {
      tx=hasX?vx:curX;
      ty=hasY?vy:curY;
      tz=hasZ?vz:curZ;
    }
    // A PICK with no Z means "use the standard weed depth" rather than
    // "stay at the current Z" -- the Pi only needs to send X and Y.
    if(action==3 && !hasZ) tz = pickZ;

    if     (action==1) doReach(tx,ty,tz);
    else if(action==2) doWeed(tx,ty,tz);
    else if(action==3) doPick(tx,ty,tz);
    else if(autoPick)  doPick(tx,ty,hasZ?tz:pickZ);   // AP1: bare coords = full cycle
    else               moveTo(tx,ty,tz,vf);
  }
}

void feedChar(char c){
  lastRxMs = millis();
  if(c=='\n'||c=='\r'){ if(lineLen) runLine(); lineLen=0; return; }
  // '0' as the first character of a line = emergency stop:
  // abort any delta move AND stop the drive wheels.
  if(c=='0'&&lineLen==0){
    abortFlag=true;
    driveCommand('0');
    sl(F("stop"));
    return;
  }
  if(lineLen<(uint8_t)(sizeof(lineBuf)-1)) lineBuf[lineLen++]=c;
}

// ==================================================================
//  SETUP / LOOP
// ==================================================================
void setup(){
  // --- delta steppers ---
  pinMode(X_STEP_PIN,OUTPUT); pinMode(Y_STEP_PIN,OUTPUT); pinMode(Z_STEP_PIN,OUTPUT);
  PORTF &= ~(STEP_F_MASK[0]|STEP_F_MASK[1]);
  PORTL &= ~STEP_L_MASK[2];
  for(uint8_t m=0;m<3;m++){
    pinMode(DIR_PIN[m],OUTPUT);
    pinMode(EN_PIN[m], OUTPUT);
    digitalWrite(EN_PIN[m],HIGH);
  }
  gearRatio = BOOT_GEAR_RATIO;
  recalcSteps();
  curX=HOME_X; curY=HOME_Y; curZ=HOME_Z;

  // --- drive motors ---
  pinMode(L_ENA,OUTPUT); pinMode(L_ENB,OUTPUT);
  pinMode(L_IN1,OUTPUT); pinMode(L_IN2,OUTPUT);
  pinMode(L_IN3,OUTPUT); pinMode(L_IN4,OUTPUT);
  pinMode(R_ENA,OUTPUT); pinMode(R_ENB,OUTPUT);
  pinMode(R_IN1,OUTPUT); pinMode(R_IN2,OUTPUT);
  pinMode(R_IN3,OUTPUT); pinMode(R_IN4,OUTPUT);
  stopAll();

  Serial.begin(USB_BAUD);
  BT.begin(BT_BAUD);
  delay(300);

  // --- gripper: attach, hold the safe centre, then release torque ---
  gripper.attach(SERVO_PIN);
  gripper.write(gripAngle);
  gripAttached = true;
  gripLastMoveMs = millis();
  delay(400);                 // let it actually get there before detaching
  if(gripAutoDetach) gripDetach();

  sl(F("Weed Robot master ready."));
  sp(F("GR=")); sp(gearRatio,6);
  sp(F("  stepsPerRad=")); sl(stepsPerRad,4);
  sp(F("drive start speed=")); sl(currentSpeed);
  sl(F("Place delta at TOP rest position, then send  SETHOME"));
  printHelp();
  setDrivers(true);
}

void loop(){
  // Replay anything that arrived while a move was blocking. Done
  // first so a queued PICK starts before any newly-arrived byte.
  if(pendLen){
    uint8_t n = pendLen; pendLen = 0;
    for(uint8_t i = 0; i < n; i++) feedChar(pendBuf[i]);
  }

  while(BT.available())     feedChar(BT.read());
  while(Serial.available()) feedChar(Serial.read());

  // Flush a lone character that arrived without a newline (phone app keys)
  if(lineLen == 1 && (millis() - lastRxMs) > SINGLE_CHAR_FLUSH_MS) runLine();

  driveWatchdog();
  gripperIdleTask();
}
