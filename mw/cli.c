#include "mw.h"

// we unset this on 'exit'
extern uint8_t cliMode;
static void cliAux(char *cmdline);
static void cliCMix(char *cmdline);
static void cliDefaults(char *cmdline);
static void cliDump(char *cmdLine);
static void cliExit(char *cmdline);
static void cliFeature(char *cmdline);
static void cliHelp(char *cmdline);
static void cliMap(char *cmdline);
static void cliMixer(char *cmdline);
static void cliProfile(char *cmdline);
static void cliSave(char *cmdline);
static void cliSet(char *cmdline);
static void cliStatus(char *cmdline);
static void cliVersion(char *cmdline);

// from sensors.c
extern uint8_t batteryCellCount;
extern uint8_t accHardware;

// from config.c RC Channel mapping
extern const char rcChannelLetters[];

// buffer
static char cliBuffer[48];
static uint32_t bufferIndex = 0;

// sync this with MultiType enum from mw.h
const char * const mixerNames[] = {
    "TRI", "QUADP", "QUADX", "BI",
    "GIMBAL", "Y6", "HEX6",
    "FLYING_WING", "Y4", "HEX6X", "OCTOX8", "OCTOFLATP", "OCTOFLATX",
    "AIRPLANE", "HELI_120_CCPM", "HELI_90_DEG", "VTAIL4", "CUSTOM", NULL
};

// sync this with AvailableFeatures enum from board.h
const char * const featureNames[] = {
    "PPM", "VBAT", "INFLIGHT_ACC_CAL", "SPEKTRUM", "MOTOR_STOP",
    "SERVO_TILT", "GYRO_SMOOTHING", "LED_RING", "GPS",
    "FAILSAFE", "SONAR", "TELEMETRY", "POWERMETER", "VARIO",
    NULL
};

// sync this with AvailableSensors enum from board.h
const char * const sensorNames[] = {
    "ACC", "BARO", "MAG", "SONAR", "GPS", NULL
};

// 
const char * const accNames[] = {
    "", "ADXL345", "MPU6050", "MMA845x", NULL
};

typedef struct {
    char *name;
    char *param;
    void (*func)(char *cmdline);
} clicmd_t;

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] = {
    { "aux", "feature_name auxflag or blank for list", cliAux },
    { "cmix", "design custom mixer", cliCMix },
    { "defaults", "reset to defaults and reboot", cliDefaults },
    { "dump", "print configurable settings in a pastable form", cliDump },
    { "exit", "", cliExit },
    { "feature", "list or -val or val", cliFeature },
    { "help", "", cliHelp },
    { "map", "mapping of rc channel order", cliMap },
    { "mixer", "mixer name or list", cliMixer },
    { "profile", "index (0 to 2)", cliProfile },
    { "save", "save and reboot", cliSave },
    { "set", "name=value or blank or * for list", cliSet },
    { "status", "show system status", cliStatus },
    { "version", "", cliVersion },
};
#define CMD_COUNT (sizeof(cmdTable) / sizeof(cmdTable[0]))

typedef enum {
    VAR_UINT8,
    VAR_INT8,
    VAR_UINT16,
    VAR_INT16,
    VAR_UINT32,
    VAR_FLOAT
} vartype_e;

typedef struct {
    const char *name;
    const uint8_t type; // vartype_e
    void *ptr;
    const int32_t min;
    const int32_t max;
} clivalue_t;

const clivalue_t valueTable[] = {
    { "looptime", VAR_UINT16, &mcfg.looptime, 0, 9000 },
    { "midrc", VAR_UINT16, &mcfg.midrc, 1200, 1700 },
    { "minthrottle", VAR_UINT16, &mcfg.minthrottle, 0, 2000 },
    { "maxthrottle", VAR_UINT16, &mcfg.maxthrottle, 0, 2000 },
    { "mincommand", VAR_UINT16, &mcfg.mincommand, 0, 2000 },
    { "mincheck", VAR_UINT16, &mcfg.mincheck, 0, 2000 },
    { "maxcheck", VAR_UINT16, &mcfg.maxcheck, 0, 2000 },
    { "retarded_arm", VAR_UINT8, &mcfg.retarded_arm, 0, 1 },
    { "motor_pwm_rate", VAR_UINT16, &mcfg.motor_pwm_rate, 50, 498 },
    { "servo_pwm_rate", VAR_UINT16, &mcfg.servo_pwm_rate, 50, 498 },
    { "serial_baudrate", VAR_UINT32, &mcfg.serial_baudrate, 1200, 115200 },
    { "gps_baudrate", VAR_UINT32, &mcfg.gps_baudrate, 1200, 115200 },
    { "spektrum_hires", VAR_UINT8, &mcfg.spektrum_hires, 0, 1 },
    { "vbatscale", VAR_UINT8, &mcfg.vbatscale, 10, 200 },
    { "vbatmaxcellvoltage", VAR_UINT8, &mcfg.vbatmaxcellvoltage, 10, 50 },
    { "vbatmincellvoltage", VAR_UINT8, &mcfg.vbatmincellvoltage, 10, 50 },
    { "power_adc_channel", VAR_UINT8, &mcfg.power_adc_channel, 0, 9 },
    { "align_gyro_x", VAR_INT8, &mcfg.align[ALIGN_GYRO][0], -3, 3 },
    { "align_gyro_y", VAR_INT8, &mcfg.align[ALIGN_GYRO][1], -3, 3 },
    { "align_gyro_z", VAR_INT8, &mcfg.align[ALIGN_GYRO][2], -3, 3 },
    { "align_acc_x", VAR_INT8, &mcfg.align[ALIGN_ACCEL][0], -3, 3 },
    { "align_acc_y", VAR_INT8, &mcfg.align[ALIGN_ACCEL][1], -3, 3 },
    { "align_acc_z", VAR_INT8, &mcfg.align[ALIGN_ACCEL][2], -3, 3 },
    { "align_mag_x", VAR_INT8, &mcfg.align[ALIGN_MAG][0], -3, 3 },
    { "align_mag_y", VAR_INT8, &mcfg.align[ALIGN_MAG][1], -3, 3 },
    { "align_mag_z", VAR_INT8, &mcfg.align[ALIGN_MAG][2], -3, 3 },
    { "acc_hardware", VAR_UINT8, &mcfg.acc_hardware, 0, 3 },
    { "moron_threshold", VAR_UINT8, &mcfg.moron_threshold, 0, 128 },
    { "gyro_lpf", VAR_UINT16, &mcfg.gyro_lpf, 0, 256 },
    { "gyro_cmpf_factor", VAR_UINT16, &mcfg.gyro_cmpf_factor, 100, 1000 },
    { "gps_type", VAR_UINT8, &mcfg.gps_type, 0, 3 },

    { "deadband", VAR_UINT8, &cfg.deadband, 0, 32 },
    { "yawdeadband", VAR_UINT8, &cfg.yawdeadband, 0, 100 },
    { "alt_hold_throttle_neutral", VAR_UINT8, &cfg.alt_hold_throttle_neutral, 1, 250 },
    { "rc_rate", VAR_UINT8, &cfg.rcRate8, 0, 250 },
    { "rc_expo", VAR_UINT8, &cfg.rcExpo8, 0, 100 },
    { "thr_mid", VAR_UINT8, &cfg.thrMid8, 0, 100 },
    { "thr_expo", VAR_UINT8, &cfg.thrExpo8, 0, 250 },
    { "roll_pitch_rate", VAR_UINT8, &cfg.rollPitchRate, 0, 100 },
    { "yawrate", VAR_UINT8, &cfg.yawRate, 0, 100 },
    { "failsafe_delay", VAR_UINT8, &cfg.failsafe_delay, 0, 200 },
    { "failsafe_off_delay", VAR_UINT8, &cfg.failsafe_off_delay, 0, 200 },
    { "failsafe_throttle", VAR_UINT16, &cfg.failsafe_throttle, 1000, 2000 },
    { "yaw_direction", VAR_INT8, &cfg.yaw_direction, -1, 1 },
    { "tri_yaw_middle", VAR_UINT16, &cfg.tri_yaw_middle, 0, 2000 },
    { "tri_yaw_min", VAR_UINT16, &cfg.tri_yaw_min, 0, 2000 },
    { "tri_yaw_max", VAR_UINT16, &cfg.tri_yaw_max, 0, 2000 },
    { "wing_left_min", VAR_UINT16, &cfg.wing_left_min, 0, 2000 },
    { "wing_left_mid", VAR_UINT16, &cfg.wing_left_mid, 0, 2000 },
    { "wing_left_max", VAR_UINT16, &cfg.wing_left_max, 0, 2000 },
    { "wing_right_min", VAR_UINT16, &cfg.wing_right_min, 0, 2000 },
    { "wing_right_mid", VAR_UINT16, &cfg.wing_right_mid, 0, 2000 },
    { "wing_right_max", VAR_UINT16, &cfg.wing_right_max, 0, 2000 },
    { "pitch_direction_l", VAR_INT8, &cfg.pitch_direction_l, -1, 1 },
    { "pitch_direction_r", VAR_INT8, &cfg.pitch_direction_r, -1, 1 },
    { "roll_direction_l", VAR_INT8, &cfg.roll_direction_l, -1, 1 },
    { "roll_direction_r", VAR_INT8, &cfg.roll_direction_r, -1, 1 },
    { "gimbal_flags", VAR_UINT8, &cfg.gimbal_flags, 0, 255},
    { "gimbal_pitch_gain", VAR_INT8, &cfg.gimbal_pitch_gain, -100, 100 },
    { "gimbal_roll_gain", VAR_INT8, &cfg.gimbal_roll_gain, -100, 100 },
    { "gimbal_pitch_min", VAR_UINT16, &cfg.gimbal_pitch_min, 100, 3000 },
    { "gimbal_pitch_max", VAR_UINT16, &cfg.gimbal_pitch_max, 100, 3000 },
    { "gimbal_pitch_mid", VAR_UINT16, &cfg.gimbal_pitch_mid, 100, 3000 },
    { "gimbal_roll_min", VAR_UINT16, &cfg.gimbal_roll_min, 100, 3000 },
    { "gimbal_roll_max", VAR_UINT16, &cfg.gimbal_roll_max, 100, 3000 },
    { "gimbal_roll_mid", VAR_UINT16, &cfg.gimbal_roll_mid, 100, 3000 },
    { "acc_lpf_factor", VAR_UINT8, &cfg.acc_lpf_factor, 0, 250 },
    { "acc_lpf_for_velocity", VAR_UINT8, &cfg.acc_lpf_for_velocity, 1, 250 },
    { "acc_trim_pitch", VAR_INT16, &cfg.angleTrim[PITCH], -300, 300 },
    { "acc_trim_roll", VAR_INT16, &cfg.angleTrim[ROLL], -300, 300 },
    { "baro_tab_size", VAR_UINT8, &cfg.baro_tab_size, 0, BARO_TAB_SIZE_MAX },
    { "baro_noise_lpf", VAR_FLOAT, &cfg.baro_noise_lpf, 0, 1 },
    { "baro_cf", VAR_FLOAT, &cfg.baro_cf, 0, 1 },
    { "mag_declination", VAR_INT16, &cfg.mag_declination, -18000, 18000 },
    { "gps_pos_p", VAR_UINT8, &cfg.P8[PIDPOS], 0, 200 },
    { "gps_pos_i", VAR_UINT8, &cfg.I8[PIDPOS], 0, 200 },
    { "gps_pos_d", VAR_UINT8, &cfg.D8[PIDPOS], 0, 200 },
    { "gps_posr_p", VAR_UINT8, &cfg.P8[PIDPOSR], 0, 200 },
    { "gps_posr_i", VAR_UINT8, &cfg.I8[PIDPOSR], 0, 200 },
    { "gps_posr_d", VAR_UINT8, &cfg.D8[PIDPOSR], 0, 200 },
    { "gps_nav_p", VAR_UINT8, &cfg.P8[PIDNAVR], 0, 200 },
    { "gps_nav_i", VAR_UINT8, &cfg.I8[PIDNAVR], 0, 200 },
    { "gps_nav_d", VAR_UINT8, &cfg.D8[PIDNAVR], 0, 200 },
    { "gps_wp_radius", VAR_UINT16, &cfg.gps_wp_radius, 0, 2000 },
    { "nav_controls_heading", VAR_UINT8, &cfg.nav_controls_heading, 0, 1 },
    { "nav_speed_min", VAR_UINT16, &cfg.nav_speed_min, 10, 2000 },
    { "nav_speed_max", VAR_UINT16, &cfg.nav_speed_max, 10, 2000 },
    { "nav_slew_rate", VAR_UINT8, &cfg.nav_slew_rate, 0, 100 },
    { "p_pitch", VAR_UINT8, &cfg.P8[PITCH], 0, 200 },
    { "i_pitch", VAR_UINT8, &cfg.I8[PITCH], 0, 200 },
    { "d_pitch", VAR_UINT8, &cfg.D8[PITCH], 0, 200 },
    { "p_roll", VAR_UINT8, &cfg.P8[ROLL], 0, 200 },
    { "i_roll", VAR_UINT8, &cfg.I8[ROLL], 0, 200 },
    { "d_roll", VAR_UINT8, &cfg.D8[ROLL], 0, 200 },
    { "p_yaw", VAR_UINT8, &cfg.P8[YAW], 0, 200 },
    { "i_yaw", VAR_UINT8, &cfg.I8[YAW], 0, 200 },
    { "d_yaw", VAR_UINT8, &cfg.D8[YAW], 0, 200 },
    { "p_alt", VAR_UINT8, &cfg.P8[PIDALT], 0, 200 },
    { "i_alt", VAR_UINT8, &cfg.I8[PIDALT], 0, 200 },
    { "d_alt", VAR_UINT8, &cfg.D8[PIDALT], 0, 200 },
    { "p_level", VAR_UINT8, &cfg.P8[PIDLEVEL], 0, 200 },
    { "i_level", VAR_UINT8, &cfg.I8[PIDLEVEL], 0, 200 },
    { "d_level", VAR_UINT8, &cfg.D8[PIDLEVEL], 0, 200 },
};

#define VALUE_COUNT (sizeof(valueTable) / sizeof(valueTable[0]))

//How much data left to read
static uint32_t cliReadLeft() {
	return 0;
}

//All input comes from there
static char cliRead() {
	return 0;
}


//All output goes through there
static void cliWrite( char c ) {

}

static void cliPrintf( const char* input, ... ) {

}

static void cliSetVar(const clivalue_t *var, const int32_t value);
static void cliPrintVar(const clivalue_t *var, uint32_t full);

static void cliPrompt(void)
{
    cliPrintf("\r\n# ");
}

static void cliAux(char *cmdline)
{
    int i, val = 0;
    uint8_t len;
    char *ptr;

    len = strlen(cmdline);
    if (len == 0) {
        // print out aux channel settings
        for (i = 0; i < CHECKBOXITEMS; i++)
        	cliPrintf("aux %u %u\r\n", i, cfg.activate[i]);
    } else {
        ptr = cmdline;
        i = atoi(ptr);
        if (i < CHECKBOXITEMS) {
            ptr = strchr(cmdline, ' ');
            val = atoi(ptr);
            cfg.activate[i] = val;
        } else {
            cliPrintf("Invalid Feature index: must be < %u\r\n", CHECKBOXITEMS);
        }
    }
}

static void cliCMix(char *cmdline)
{
    int i, check = 0;
    int num_motors = 0;
    uint8_t len;
    float mixsum[3];
    char *ptr;

    len = strlen(cmdline);

    if (len == 0) {
        cliPrintf("Custom mixer: \r\nMotor\tThr\tRoll\tPitch\tYaw\r\n");
        for (i = 0; i < MAX_MOTORS; i++) {
            if (mcfg.customMixer[i].throttle == 0.0f)
                break;
            num_motors++;
            cliPrintf("#%d:\t", i + 1);
            cliPrintf("%f\t", mcfg.customMixer[i].throttle );
            cliPrintf("%f\t", mcfg.customMixer[i].roll );
            cliPrintf("%f\t", mcfg.customMixer[i].pitch );
            cliPrintf("%f\r\n", mcfg.customMixer[i].yaw );
        }
        mixsum[0] = mixsum[1] = mixsum[2] = 0.0f;
        for (i = 0; i < num_motors; i++) {
            mixsum[0] += mcfg.customMixer[i].roll;
            mixsum[1] += mcfg.customMixer[i].pitch;
            mixsum[2] += mcfg.customMixer[i].yaw;
        }
        cliPrintf("Sanity check:\t");
        for (i = 0; i < 3; i++)
            cliPrintf(fabs(mixsum[i]) > 0.01f ? "NG\t" : "OK\t");
        cliPrintf("\r\n");
        return;
    } else if (strncasecmp(cmdline, "reset", 5) == 0) {
        // erase custom mixer
        for (i = 0; i < MAX_MOTORS; i++)
            mcfg.customMixer[i].throttle = 0.0f;
    } else if (strncasecmp(cmdline, "load", 4) == 0) {
        ptr = strchr(cmdline, ' ');
        if (ptr) {
            len = strlen(++ptr);
            for (i = 0; ; i++) {
                if (mixerNames[i] == NULL) {
                    cliPrintf("Invalid mixer type...\r\n");
                    break;
                }
                if (strncasecmp(ptr, mixerNames[i], len) == 0) {
                    mixerLoadMix(i);
                    cliPrintf("Loaded %s mix...\r\n", mixerNames[i]);
                    cliCMix("");
                    break;
                }
            }
        }
    } else {
        ptr = cmdline;
        i = atoi(ptr); // get motor number
        if (--i < MAX_MOTORS) {
            ptr = strchr(ptr, ' ');
            if (ptr) {
                mcfg.customMixer[i].throttle = atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                mcfg.customMixer[i].roll = atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                mcfg.customMixer[i].pitch = atof(++ptr);
                check++;
            }
            ptr = strchr(ptr, ' ');
            if (ptr) {
                mcfg.customMixer[i].yaw = atof(++ptr);
                check++;
            }
            if (check != 4) {
                cliPrintf("Wrong number of arguments, needs idx thr roll pitch yaw\r\n");
            } else {
                cliCMix("");
            }
        } else {
            cliPrintf("Motor number must be between 1 and %d\r\n", MAX_MOTORS);
        }
    }
}

static void cliDefaults(char *cmdline)
{
    cliPrintf("Resetting to defaults...\r\n");
    checkFirstTime(true);
    cliPrintf("Rebooting...");
    delayMilli( 10 );
    boardReset();
}

static void cliDump(char *cmdline)
{
    
    int i;
    char buf[16];
    float thr, roll, pitch, yaw;
    uint32_t mask;
    const clivalue_t *setval;

    cliPrintf("Current Config: Copy everything below here...\r\n");

    // print out aux switches
    cliAux("");

    // print out current motor mix
    cliPrintf("mixer %s\r\n", mixerNames[mcfg.mixerConfiguration - 1]);

    // print custom mix if exists
    if (mcfg.customMixer[0].throttle != 0.0f) {
        for (i = 0; i < MAX_MOTORS; i++) {
            if (mcfg.customMixer[i].throttle == 0.0f)
                break;
            thr = mcfg.customMixer[i].throttle;
            roll = mcfg.customMixer[i].roll;
            pitch = mcfg.customMixer[i].pitch;
            yaw = mcfg.customMixer[i].yaw;
            cliPrintf("cmix %d", i + 1);
            if (thr < 0) 
                cliPrintf(" ");
            cliPrintf( "%f", thr );
            if (roll < 0) 
                cliPrintf(" ");
            cliPrintf( "%f", roll );;
            if (pitch < 0) 
                cliPrintf(" ");
            cliPrintf( "%f", pitch );
            if (yaw < 0) 
                cliPrintf(" ");
            cliPrintf( "%f\r\n", yaw );
        }   
        cliPrintf("cmix %d 0 0 0 0\r\n", i + 1);
    }

    // print enabled features
    mask = featureMask();
    for (i = 0; ; i++) { // disable all feature first
        if (featureNames[i] == NULL)
            break;
        cliPrintf("feature -%s\r\n", featureNames[i]);
    }
    for (i = 0; ; i++) {  // reenable what we want.
        if (featureNames[i] == NULL)
            break;
        if (mask & (1 << i))
            cliPrintf("feature %s\r\n", featureNames[i]);
    }

    // print RC MAPPING
    for (i = 0; i < 8; i++)
        buf[mcfg.rcmap[i]] = rcChannelLetters[i];
    buf[i] = '\0';
    cliPrintf("map %s\r\n", buf);

    // print settings
    for (i = 0; i < VALUE_COUNT; i++) {
        setval = &valueTable[i];
        cliPrintf("set %s = ", valueTable[i].name);
        cliPrintVar(setval, 0);
        cliPrintf("\r\n");
    }
}

static void cliExit(char *cmdline)
{
    cliPrintf("\r\nLeaving CLI mode...\r\n");
    memset(cliBuffer, 0, sizeof(cliBuffer));
    bufferIndex = 0;
    cliMode = 0;
    // save and reboot... I think this makes the most sense
    cliSave(cmdline);
}

static void cliFeature(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    uint32_t mask;

    len = strlen(cmdline);
    mask = featureMask();

    if (len == 0) {
        cliPrintf("Enabled features: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            if (mask & (1 << i))
                cliPrintf("%s ", featureNames[i]);
        }
        cliPrintf("\r\n");
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrintf("Available features: ");
        for (i = 0; ; i++) {
            if (featureNames[i] == NULL)
                break;
            cliPrintf("%s ", featureNames[i]);
        }
        cliPrintf("\r\n");
        return;
    } else {
        bool remove = false;
        if (cmdline[0] == '-') {
            // remove feature
            remove = true;
            cmdline++; // skip over -
            len--;
        }

        for (i = 0; ; i++) {
            if (featureNames[i] == NULL) {
                cliPrintf("Invalid feature name...\r\n");
                break;
            }
            if (strncasecmp(cmdline, featureNames[i], len) == 0) {
                if (remove) {
                    featureClear(1 << i);
                    cliPrintf("Disabled ");
                } else {
                    featureSet(1 << i);
                    cliPrintf("Enabled ");
                }
                cliPrintf("%s\r\n", featureNames[i]);
                break;
            }
        }
    }
}

static void cliHelp(char *cmdline)
{
    uint32_t i = 0;

    cliPrintf("Available commands:\r\n");
    for (i = 0; i < CMD_COUNT; i++)
        cliPrintf("%s\t%s\r\n", cmdTable[i].name, cmdTable[i].param);
}

static void cliMap(char *cmdline)
{
    uint32_t len;
    uint32_t i;
    char out[9];

    len = strlen(cmdline);

    if (len == 8) {
        // uppercase it
        for (i = 0; i < 8; i++)
            cmdline[i] = toupper(cmdline[i]);
        for (i = 0; i < 8; i++) {
            if (strchr(rcChannelLetters, cmdline[i]) && !strchr(cmdline + i + 1, cmdline[i]))
                continue;
            cliPrintf("Must be any order of AETR1234\r\n");
            return;
        }
        parseRcChannels(cmdline);
    }
    cliPrintf("Current assignment: ");
    for (i = 0; i < 8; i++)
        out[mcfg.rcmap[i]] = rcChannelLetters[i];
    out[i] = '\0';
    cliPrintf("%s\r\n", out);
}

static void cliMixer(char *cmdline)
{
    int i;
    int len;

    len = strlen(cmdline);

    if (len == 0) {
        cliPrintf("Current mixer: %s\r\n", mixerNames[mcfg.mixerConfiguration - 1]);
        return;
    } else if (strncasecmp(cmdline, "list", len) == 0) {
        cliPrintf("Available mixers: ");
        for (i = 0; ; i++) {
            if (mixerNames[i] == NULL)
                break;
            cliPrintf("%s ", mixerNames[i]);
        }
        cliPrintf("\r\n");
        return;
    }

    for (i = 0; ; i++) {
        if (mixerNames[i] == NULL) {
            cliPrintf("Invalid mixer type...\r\n");
            break;
        }
        if (strncasecmp(cmdline, mixerNames[i], len) == 0) {
            mcfg.mixerConfiguration = i + 1;
            cliPrintf("Mixer set to %s\r\n", mixerNames[i]);
            break;
        }
    }
}

static void cliProfile(char *cmdline)
{
    uint8_t len;
    int i;

    len = strlen(cmdline);
    if (len == 0) {
        cliPrintf("Current profile: %d\r\n", mcfg.current_profile);
        return;
    } else {
        i = atoi(cmdline);
        if (i >= 0 && i <= 2) {
            mcfg.current_profile = i;
            writeEEPROM(0, false);
            cliProfile("");
        }
    }
}

static void cliSave(char *cmdline)
{
    cliPrintf("Saving...");
    writeEEPROM(0, true);
    cliPrintf("\r\nRebooting...");
    delayMilli(10);
    boardReset();
}

static void cliPrintVar(const clivalue_t *var, uint32_t full)
{
    int32_t value = 0;

    switch (var->type) {
        case VAR_UINT8:
            value = *(uint8_t *)var->ptr;
            break;

        case VAR_INT8:
            value = *(int8_t *)var->ptr;
            break;

        case VAR_UINT16:
            value = *(uint16_t *)var->ptr;
            break;

        case VAR_INT16:
            value = *(int16_t *)var->ptr;
            break;

        case VAR_UINT32:
            value = *(uint32_t *)var->ptr;
            break;

        case VAR_FLOAT:
            cliPrintf("%f", *(float *)var->ptr );
            if (full) {
                cliPrintf(" %f", (float)var->min );
                cliPrintf(" %f", (float)var->max );
            }
            return; // return from case for float only
    }
    cliPrintf("%d", value);
    if (full)
        cliPrintf(" %d %d", var->min, var->max);
}

static void cliSetVar(const clivalue_t *var, const int32_t value)
{
    switch (var->type) {
        case VAR_UINT8:
        case VAR_INT8:
            *(char *)var->ptr = (char)value;
            break;

        case VAR_UINT16:
        case VAR_INT16:
            *(short *)var->ptr = (short)value;
            break;

        case VAR_UINT32:
            *(int *)var->ptr = (int)value;
            break;

        case VAR_FLOAT:
            *(float *)var->ptr = *(float *)&value;
            break;
    }
}

static void cliSet(char *cmdline)
{
    uint32_t i;
    uint32_t len;
    const clivalue_t *val;
    char *eqptr = NULL;
    int32_t value = 0;
    float valuef = 0;

    len = strlen(cmdline);

    if (len == 0 || (len == 1 && cmdline[0] == '*')) {
        cliPrintf("Current settings: \r\n");
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            cliPrintf("%s = ", valueTable[i].name);
            cliPrintVar(val, len); // when len is 1 (when * is passed as argument), it will print min/max values as well, for gui
            cliPrintf("\r\n");
        }
    } else if ( (eqptr = strchr(cmdline, '=')) ) {
        // has equal, set var
        eqptr++;
        len--;
        value = atoi(eqptr);
        valuef = atof(eqptr);
        for (i = 0; i < VALUE_COUNT; i++) {
            val = &valueTable[i];
            if (strncasecmp(cmdline, valueTable[i].name, strlen(valueTable[i].name)) == 0) {
                if (valuef >= valueTable[i].min && valuef <= valueTable[i].max) { // here we compare the float value since... it should work, RIGHT?
                    cliSetVar(val, valueTable[i].type == VAR_FLOAT ? *(uint32_t *)&valuef : value); // this is a silly dirty hack. please fix me later.
                    cliPrintf("%s set to ", valueTable[i].name);
                    cliPrintVar(val, 0);
                } else {
                    cliPrintf("ERR: Value assignment out of range\r\n");
                }
                return;
            }
        }
        cliPrintf("ERR: Unknown variable name\r\n");
    }
}

static void cliStatus(char *cmdline)
{
//    uint8_t i;
//    uint32_t mask;

    cliPrintf("System Uptime: %d seconds, Voltage: %d * 0.1V (%dS battery)\r\n",
        timeMilli() / 1000, vbat, batteryCellCount);
#if 0
    mask = sensorsMask();
    cliPrintf("CPU %dMHz, detected sensors: ", (SystemCoreClock / 1000000));
    for (i = 0; ; i++) {
        if (sensorNames[i] == NULL)
            break;
        if (mask & (1 << i))
            cliPrintf("%s ", sensorNames[i]);
    }
    if (sensors(SENSOR_ACC)) {
        cliPrintf("ACCHW: %s", accNames[accHardware]);
        if (accHardware == ACC_MPU6050)
            cliPrintf(".%c", mcfg.mpu6050_scale ? 'o' : 'n');
    }
#endif
    cliPrintf("\r\n");

    cliPrintf("Cycle Time: %d, I2C Errors: %d, config size: %d\r\n", cycleTime, i2cGetErrorCounter(), sizeof(master_t));
}

static void cliVersion(char *cmdline)
{
    cliPrintf("Afro32 CLI version 2.1 " __DATE__ " / " __TIME__);
}

void cliProcess(void)
{
    if (!cliMode) {
        cliMode = 1;
        cliPrintf("\r\nEntering CLI Mode, type 'exit' to return, or 'help'\r\n");
        cliPrompt();
    }

    while ( cliReadLeft() ) {
        uint8_t c = cliRead();
        if (c == '\t' || c == '?') {
            // do tab completion
            const clicmd_t *cmd, *pstart = NULL, *pend = NULL;
            int i = bufferIndex;
            for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++) {
                if (bufferIndex && (strncasecmp(cliBuffer, cmd->name, bufferIndex) != 0))
                    continue;
                if (!pstart)
                    pstart = cmd;
                pend = cmd;
            }
            if (pstart) {    /* Buffer matches one or more commands */
                for (; ; bufferIndex++) {
                    if (pstart->name[bufferIndex] != pend->name[bufferIndex])
                        break;
                    if (!pstart->name[bufferIndex]) {
                        /* Unambiguous -- append a space */
                        cliBuffer[bufferIndex++] = ' ';
                        break;
                    }
                    cliBuffer[bufferIndex] = pstart->name[bufferIndex];
                }
            }
            if (!bufferIndex || pstart != pend) {
                /* Print list of ambiguous matches */
                cliPrintf("\r\033[K");
                for (cmd = pstart; cmd <= pend; cmd++) {
                    cliPrintf( "%s\t", cmd->name);
                }
                cliPrompt();
                i = 0;    /* Redraw prompt */
            }
            for (; i < bufferIndex; i++)
                cliWrite(cliBuffer[i]);
        } else if (!bufferIndex && c == 4) {
            cliExit(cliBuffer);
            return;
        } else if (c == 12) {
            // clear screen
            cliPrintf("\033[2J\033[1;1H");
            cliPrompt();
        } else if (bufferIndex && (c == '\n' || c == '\r')) {
            // enter pressed
            const clicmd_t *cmd = NULL;
            cliPrintf("\r\n");
            cliBuffer[bufferIndex] = 0; // null terminate
            
            for (cmd = cmdTable; cmd < cmdTable + CMD_COUNT; cmd++) {
            	if ( !strcasecmp( cliBuffer, cmd->name) ) {
            		cmd->func( cliBuffer + strlen(cmd->name) + 1 );
            		//Indicate the command was
            		cmd = NULL;
            		break;
            	}
            }

            //Check if a command was found else show help info
            if ( cmd ) {
                cliPrintf( "ERR: Unknown command, try 'help'" );
            }
            
            //Reset the buffer to receive a new command
            cliBuffer[0] = 0;
            bufferIndex = 0;

            // 'exit' will reset this flag, so we don't need to print prompt again
            if (!cliMode)
                return;
            cliPrompt();
        } else if (c == 127) {
            // backspace
            if (bufferIndex) {
                cliBuffer[--bufferIndex] = 0;
                cliPrintf("\010 \010");
            }
        } else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126) {
            if (!bufferIndex && c == 32)
                continue;
            //Add character to parser buffer and print it
            cliBuffer[bufferIndex++] = c;
            cliWrite(c);
        }
    }
}
