#include <initializer_list>
#include <string.h>
#include <stdio.h>

#include "activity_lib_constants.h"
#include "allowed_params.hpp"

namespace Activity {

bool volatile allowed_params_inited = false;
AllowedParam ALLOWED_PARAMS[ALLOWED_PARAM_COUNT];

AllowedParam::AllowedParam() {
    name = nullptr;
    display_name = nullptr;
    display_values = nullptr;
}

AllowedParam::AllowedParam(int _id, const char * _name, param_target_device _target_device, const char * _units, const char * _display_name, int8_t _visible_if_afol_mode, const char * const * _display_values) {

    id = _id;
    name = _name;

    target_device = _target_device;

    units = _units;
    display_name = _display_name;

    visible_if_afol_mode = _visible_if_afol_mode;

    display_values = _display_values;
    for (display_value_count = 0; display_values[display_value_count]!=nullptr; display_value_count++);
}

const char * const DISPLAY_VALUES_EMPTY[]         = {nullptr};
const char * const DISPLAY_VALUES_NAV_AFOL_MODE[] = {"Fixed", "Path", "Line", "Adaptive", "Circle", nullptr};
const char * const DISPLAY_VALUES_ON_OFF[]        = {"Off", "On", nullptr};
const char * const DISPLAY_VALUES_YES_NO[]        = {"No", "Yes", nullptr};
const char * const DISPLAY_VALUES_REACTION[]      = {"Easy", "Medium", "Aggresive", nullptr};
const char * const DISPLAY_VALUES_RTH_SPOT[]      = {"ERROR", "Home", "Spot", nullptr};
const char * const DISPLAY_VALUES_ADAPTIVE_ANG[]  = {"Front", "Front R", "R", "Rear R", "Rear", "Rear L", "Left", "Front L", nullptr};

bool 
init_allowed_params() {

    ALLOWED_PARAMS[0] =  AllowedParam(0,     "A_ACTIVITY",           ALL,  "",    "Activity",              -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[1] =  AllowedParam(1,     "NAV_AFOL_MODE",        DOG,  "",    "Flight mode",           -1, DISPLAY_VALUES_NAV_AFOL_MODE);
    ALLOWED_PARAMS[2] =  AllowedParam(16,    "A_SAH_NO_SPOT",        DOG,  "",    "Enforce Home",          -1, DISPLAY_VALUES_ON_OFF);
    ALLOWED_PARAMS[3] =  AllowedParam(17,    "OFF_FR_ADD_ANG",       DOG,  "",    "Camera angle",           3, DISPLAY_VALUES_ADAPTIVE_ANG);
    ALLOWED_PARAMS[4] =  AllowedParam(18,    "CBP_MAX_INIT_SPD",     DOG,  "m/s", "Go to line speed",       2, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[5] =  AllowedParam(3,     "NAV_TAKEOFF_ALT",      DOG,  "m",   "Takeoff alt",           -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[6] =  AllowedParam(6,     "A_INIT_POS_D",         DOG,  "m",   "Follow distance",       -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[7] =  AllowedParam(2,     "A_BSC_SAF_ACT",        DOG,  "",    "Landing mode",          -1, DISPLAY_VALUES_RTH_SPOT);
    ALLOWED_PARAMS[8] =  AllowedParam(19,    "A_FOL_IMDTLY",         DOG,  "",    "Follow after takeoff",  -1, DISPLAY_VALUES_YES_NO);
    ALLOWED_PARAMS[9] =  AllowedParam(4,     "RTL_RET_ALT",          DOG,  "m",   "Return altitude",       -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[10] = AllowedParam(8,     "FOL_RPT_ALT",          DOG,  "m",   "Follow terrain",        -1, DISPLAY_VALUES_ON_OFF);
    ALLOWED_PARAMS[11] = AllowedParam(13,    "FOL_VEL_FF_XY",        DOG,  "",    "Acceleration",          -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[12] = AllowedParam(12,    "FOL_LPF_XY",           DOG,  "",    "Reaction",              -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[13] = AllowedParam(5,     "A_INIT_POS_U",         DOG,  "",    "Initial distance",      -1, DISPLAY_VALUES_ON_OFF);
    ALLOWED_PARAMS[14] = AllowedParam(7,     "V_REACTION",           DOG,  "",    "Reaction",              -1, DISPLAY_VALUES_REACTION);
    ALLOWED_PARAMS[15] = AllowedParam(9,     "SENS_SON_MIN",         DOG,  "m",   "Lidar alt",             -1, DISPLAY_VALUES_NAV_AFOL_MODE);
    ALLOWED_PARAMS[16] = AllowedParam(11,    "AIRD_TRAJ_RAD",        ALL,  "m",   "Follow Path rad",        1, DISPLAY_VALUES_ON_OFF);
    ALLOWED_PARAMS[17] = AllowedParam(20,    "FOL_FF_GRAD_END",      DOG,  "",    "Gradient END",          -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[18] = AllowedParam(21,    "FOL_FF_GRAD_STRT",     DOG,  "",    "Gradient START",        -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[19] = AllowedParam(22,    "FOL_FF_GRAD_USE",      DOG,  "",    "Gradien on/off",        -1, DISPLAY_VALUES_YES_NO);
    ALLOWED_PARAMS[20] = AllowedParam(23,    "FOL_USE_ALT",          DOG,  "",    "Use follow alt",        -1, DISPLAY_VALUES_YES_NO);
    ALLOWED_PARAMS[21] = AllowedParam(24,    "INAV_INIT_EPH",        DOG,  "",    "INAV INIT EPH",         -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[22] = AllowedParam(25,    "INAV_INIT_EPV",        DOG, "",     "INAV INIT EPV",         -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[23] = AllowedParam(26,    "INAV_INIT_WAIT",       DOG, "",     "INAV INIT WAIT",        -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[24] = AllowedParam(27,    "MPC_PITCH_LPF",        DOG, "",     "MPC PITCH LPF",         -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[25] = AllowedParam(28,    "MPC_XY_FF",            DOG, "",     "MPC XY FF",             -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[26] = AllowedParam(29,    "MPC_XY_P",             DOG, "",    "MPC XY P",               -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[27] = AllowedParam(30,    "MPC_XY_VEL_D",         DOG, "",    "MPC XY VEL D",           -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[28] = AllowedParam(31,    "MPC_XY_VEL_I",         DOG, "",    "MPC XY VEL I",           -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[29] = AllowedParam(32,    "MPC_XY_VEL_MAX",       DOG, "",    "MPC XY VEL MAX",         -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[30] = AllowedParam(33,    "MPC_XY_VEL_P",         DOG, "",    "MPC XY VEL P",           -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[31] = AllowedParam(34,    "MPC_Z_FF",             DOG, "",    "MPC Z FF",               -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[32] = AllowedParam(35,    "MPC_Z_P",              DOG, "",    "MPC Z P",                -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[33] = AllowedParam(36,    "MPC_Z_VEL_D",          DOG, "",    "MPC Z VEL D",            -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[34] = AllowedParam(37,    "MPC_Z_VEL_I",          DOG, "",    "MPC Z VEL I",            -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[35] = AllowedParam(38,    "MPC_Z_VEL_MAX",        DOG, "",    "MPC Z VEL MAX",          -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[36] = AllowedParam(39,    "MPC_Z_VEL_P",          DOG, "",    "MPC Z VEL P",            -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[37] = AllowedParam(40,    "PAFOL_GT_AC_DST",      DOG, "",    "PAFOL GT AC DST",        -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[38] = AllowedParam(41,    "SDLOG_ON_BOOT",        DOG, "",    "SDLOG ON BOOT",          -1, DISPLAY_VALUES_ON_OFF);
    ALLOWED_PARAMS[39] = AllowedParam(42,    "SENS_SON_ON",          DOG, "",    "SENS SON ON",            -1, DISPLAY_VALUES_ON_OFF);
    ALLOWED_PARAMS[40] = AllowedParam(43,    "UBX_ENABLE_SBAS",      DOG, "",    "UBX ENABLE SBAS",        -1, DISPLAY_VALUES_YES_NO);
    ALLOWED_PARAMS[41] = AllowedParam(44,    "UBX_GPS_MAX_CHN",      DOG, "",    "UBX GPS MAX CHN",        -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[42] = AllowedParam(45,    "UBX_GPS_MIN_CHN",      DOG, "",    "UBX GPS MAX CHN",        -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[43] = AllowedParam(46,    "INAV_W_Z_GPS_P",       DOG, "",    "INAV W Z GPS P",         -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[44] = AllowedParam(47,    "INAV_W_Z_BARO",        DOG, "",    "INAV W Z BARO",          -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[45] = AllowedParam(48,    "NAV_CP_FIR_AL",        DOG, "",    "INAV W Z BARO",          -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[46] = AllowedParam(49,    "NAV_CP_FIR_LA",        DOG, "",    "INAV W Z BARO",          -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[47] = AllowedParam(50,    "NAV_CP_FIR_LO",        DOG, "",    "INAV W Z BARO",          -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[48] = AllowedParam(51,    "NAV_CP_LAS_AL",        DOG, "",    "INAV W Z BARO",          -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[49] = AllowedParam(52,    "NAV_CP_LAS_LA",        DOG, "",    "INAV W Z BARO",          -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[50] = AllowedParam(53,    "NAV_CP_LAS_LO",        DOG, "",    "INAV W Z BARO",          -1, DISPLAY_VALUES_EMPTY);
    ALLOWED_PARAMS[51] = AllowedParam(54,    "BAT_FLAT_LVL",         DOG, "",    "BAT FLAT LVL",           -1, DISPLAY_VALUES_EMPTY);


    allowed_params_inited = true;
    return true;
}
}
// end of namespace Activity

