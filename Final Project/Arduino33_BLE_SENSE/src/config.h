// Motor pins
#define enA 7   // Left PWM
#define in1 12   // Left DIR 1
#define in2 8   // Left DIR 2

#define enB 2   // Right PWM
#define in3 4   // Right DIR 1
#define in4 3   // Right DIR 2

#define MAX_SPEED 100       // Adjusted PWM—try 255 if needed
#define ROTATION_TIME 610  // ~1220 ms for ~1560 ticks (1 rev)
#define LINEAR_TIME 1220  // ~1220 ms for ~1560 ticks (1 rev)

// Robot parameters
const float WHEEL_RADIUS = 0.0485;  // In meters
const float WHEEL_BASE = 0.17;       // m (assumed—adjust)
const float TICKS_PER_REV = 1560;   // 19.5:1 × 80