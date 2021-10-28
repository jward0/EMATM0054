#define BUTTON_B 30
#define YELLOW_LED 13

#define PID_UPDATE 10
#define LS_UPDATE 20

Motor_c motors;
lineSensors_c line_sensors;
Kinematics_c robot_kinematics;
PID_c left_controller;
PID_c right_controller;
PID_c heading_controller;

float left_rotational_velocity = 0;
float right_rotational_velocity = 0;

long last_e_left;
long last_e_right;
float last_v_left = 0;
float last_v_right = 0;

unsigned long current_ts;
unsigned long kinematics_ts;
unsigned long pid_ts;
unsigned long elapsed_t;

unsigned long large_timer;
unsigned long large_timer_start;

float heading_demand = 0;
float left_demand = 0;
float right_demand = 0;

bool robot_initialised = false;
bool traversing_break = false;
bool break_traversed = false;

unsigned long ls_read_times[NB_LS_PINS];
float calibrated_read_times[NB_LS_PINS];
unsigned long ls_offsets[NB_LS_PINS];
float ls_scales[NB_LS_PINS];

int left_drive = 0;
int right_drive = 0;

enum state_enum {
  INITIALISATION,
  SEARCHING,
  FOLLOWING,
  LINE_END,
  CONTROLLED_ROTATING,
  CONTROLLED_DRIVING,
  FINISHED,
  S_ERROR
};

state_enum state;