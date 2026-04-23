#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * motor 灞傚彧璐熻矗涓変欢浜嬶細
 * 1. 璁板綍姣忎釜鐢垫満鎸傚湪鍝竴璺?CAN銆侀€昏緫 ID 鏄灏戯紱
 * 2. 鎶?HAL CAN 鎺ユ敹鍒扮殑鍙嶉甯у垎鍙戝埌瀵瑰簲鐢垫満瀵硅薄锛? * 3. 鎸?DJI / LK 鍗忚鎵撳寘鎺у埗甯э紝骞堕€氳繃 HAL_CAN_AddTxMessage() 鍙戝嚭銆? *
 * 涓婂眰浠诲姟涓嶉渶瑕佸叧蹇冨叿浣?StdId锛屼篃涓嶉渶瑕佸湪涓柇閲屾墜鍔ㄨВ鏋?CAN 鏁版嵁銆? */

/* DJI 鐢垫満缂栫爜鍣ㄤ竴鍦?8192 涓鏁般€侴M6020 / M3508 閮芥寜杩欎釜鑼冨洿瑙ｆ瀽銆?*/
#define MOTOR_DJI_ENCODER_RANGE                 8192U
#define MOTOR_DJI_ENCODER_HALF_RANGE            4096
#define MOTOR_DJI_ENCODER_DEG_PER_TICK          (360.0f / (float)MOTOR_DJI_ENCODER_RANGE)

/* LK 鐢垫満鐘舵€?2 甯ч噷鐨勫崟鍦堢紪鐮佸櫒閫氬父鏄?16 bit锛屾寜 65536 涓鏁拌В鏋愪竴鍦堛€?*/
#define MOTOR_LK_ENCODER_RANGE                  65536U
#define MOTOR_LK_ENCODER_HALF_RANGE             32768
#define MOTOR_LK_ENCODER_DEG_PER_TICK           (360.0f / (float)MOTOR_LK_ENCODER_RANGE)

/* 瓒呰繃杩欎釜鏃堕棿娌℃湁鏀跺埌鍙嶉甯э紝灏辫涓虹數鏈烘帀绾裤€傚崟浣?ms锛屾潵婧愭槸 HAL_GetTick()銆?*/
#define MOTOR_ONLINE_TIMEOUT_MS                 100U

/*
 * GM6020 鏀寔涓ょ被鎺у埗甯э細
 * - 鐢垫祦鐜細0x1FE / 0x2FE锛岀粰瀹氬€间竴鑸寜 -16384 ~ 16384 浣跨敤锛? * - 鐢靛帇鐜細0x1FF / 0x2FF锛岀粰瀹氬€间竴鑸寜 -30000 ~ 30000 浣跨敤銆? */
typedef enum
{
    MOTOR_GM6020_CONTROL_CURRENT = 0U,
    MOTOR_GM6020_CONTROL_VOLTAGE = 1U
} gm6020_control_mode_t;

/* DJI 鐢垫満鍙嶉鐨勯€氱敤瑙ｆ瀽缁撴灉銆?*/
typedef struct
{
    uint8_t initialized;                         /* 鏄惁宸茬粡鐢ㄧ涓€甯у缓绔嬬紪鐮佸櫒闆剁偣銆?*/
    uint16_t raw_ecd;                            /* 褰撳墠鍗曞湀缂栫爜鍣ㄥ師濮嬪€硷紝鑼冨洿 0~8191銆?*/
    uint16_t last_raw_ecd;                       /* 涓婁竴甯у崟鍦堢紪鐮佸櫒鍘熷鍊笺€?*/
    uint16_t ecd_bias;                           /* 涓婄數鍚庣涓€甯х紪鐮佸櫒鍊硷紝浣滀负鐩稿闆剁偣銆?*/
    int16_t ecd_delta;                           /* 鏈抚鐩稿涓婁竴甯х殑鏈€鐭紪鐮佸櫒澧為噺銆?*/
    int32_t round_count;                         /* 鏍规嵁缂栫爜鍣ㄨ法闆跺垽鏂疮璁″嚭鐨勫湀鏁般€?*/
    int32_t total_ecd;                           /* 浠庝笂鐢甸浂鐐瑰紑濮嬬疮璁＄殑杩炵画缂栫爜鍣ㄨ鏁般€?*/
    int16_t speed_rpm;                           /* 鐢佃皟鍙嶉杞€燂紝鍗曚綅 rpm銆?*/
    int16_t given_current;                       /* 鐢佃皟鍙嶉杞煩鐢垫祦锛屽崟浣嶇敱鐢佃皟鍗忚瀹氫箟銆?*/
    uint8_t temperature;                         /* 鐢佃皟鍙嶉娓╁害銆?*/
    float angle_deg;                             /* 浠?ecd_bias 涓洪浂鐐圭殑鐩稿瑙掑害锛屽崟浣?deg銆?*/
    float total_angle_deg;                       /* 浠庝笂鐢靛紑濮嬬疮璁＄殑杩炵画瑙掑害锛屽崟浣?deg銆?*/
    float speed_dps;                             /* 鐢?rpm 鎹㈢畻鍑虹殑瑙掗€熷害锛屽崟浣?deg/s銆?*/
} dji_motor_feedback_t;

/* GM6020 鏈嶅姟瀵硅薄銆傜敱涓婂眰浠诲姟鎸佹湁锛宮otor 灞傚湪 CAN 涓柇鍥炶皟閲屽埛鏂板弽棣堛€?*/
typedef struct
{
    CAN_HandleTypeDef *hcan;                     /* 鐢垫満鎵€鍦?CAN 鍙ユ焺锛屼緥濡?&hcan1 / &hcan2銆?*/
    uint8_t motor_id;                            /* 鐢垫満鎷ㄧ爜 ID锛岄€氬父涓?1~7銆?*/
    uint32_t feedback_std_id;                    /* 鍙嶉甯ф爣鍑?ID锛孏M6020 ID1 瀵瑰簲 0x205銆?*/
    gm6020_control_mode_t control_mode;          /* 褰撳墠娉ㄥ唽鎴愮數娴佺幆杩樻槸鐢靛帇鐜€?*/
    dji_motor_feedback_t feedback;               /* 瀹屾暣 DJI 鍙嶉瑙ｆ瀽缁撴灉銆?*/
    int16_t output;                              /* 涓婂眰缂撳瓨鐨勬帶鍒惰緭鍑猴紝鍙戦€佹椂鎵撳寘杩涙帶鍒跺抚銆?*/
    volatile uint8_t online;                     /* 鏈€杩?MOTOR_ONLINE_TIMEOUT_MS 鍐呮槸鍚︽敹鍒板弽棣堛€?*/
    volatile uint32_t last_update_tick;          /* 鏈€杩戜竴娆℃敹鍒板弽棣堢殑 HAL tick銆?*/
    volatile int16_t speed_rpm;                  /* 渚挎嵎瀛楁锛氬弽棣堣浆閫?rpm銆?*/
    volatile int16_t given_current;              /* 渚挎嵎瀛楁锛氬弽棣堣浆鐭╃數娴併€?*/
    volatile uint8_t temperature;                /* 渚挎嵎瀛楁锛氬弽棣堟俯搴︺€?*/
    volatile float angle_deg;                    /* 渚挎嵎瀛楁锛氱浉瀵硅搴?deg銆?*/
    volatile float total_angle_deg;              /* 渚挎嵎瀛楁锛氳繛缁疮璁¤搴?deg銆?*/
    volatile float speed_dps;                    /* 渚挎嵎瀛楁锛氳閫熷害 deg/s銆?*/
} gm6020_service_t;

/* M3508 / M2006 浣跨敤 C620 / C610 鐢佃皟锛屽弽棣堟牸寮忎笌 DJI 閫氱敤鏍煎紡涓€鑷淬€?*/
typedef struct
{
    CAN_HandleTypeDef *hcan;                     /* 鐢垫満鎵€鍦?CAN 鍙ユ焺銆?*/
    uint8_t motor_id;                            /* 鐢垫満鎷ㄧ爜 ID锛岄€氬父涓?1~8銆?*/
    uint32_t feedback_std_id;                    /* 鍙嶉甯ф爣鍑?ID锛孖D1 瀵瑰簲 0x201銆?*/
    dji_motor_feedback_t feedback;               /* 瀹屾暣 DJI 鍙嶉瑙ｆ瀽缁撴灉銆?*/
    int16_t output;                              /* 涓婂眰缂撳瓨鐨勭數娴佺幆杈撳嚭銆?*/
    volatile uint8_t online;                     /* 鏈€杩?MOTOR_ONLINE_TIMEOUT_MS 鍐呮槸鍚︽敹鍒板弽棣堛€?*/
    volatile uint32_t last_update_tick;          /* 鏈€杩戜竴娆℃敹鍒板弽棣堢殑 HAL tick銆?*/
    volatile int16_t speed_rpm;                  /* 渚挎嵎瀛楁锛氬弽棣堣浆閫?rpm銆?*/
    volatile int16_t given_current;              /* 渚挎嵎瀛楁锛氬弽棣堣浆鐭╃數娴併€?*/
    volatile uint8_t temperature;                /* 渚挎嵎瀛楁锛氬弽棣堟俯搴︺€?*/
    volatile float angle_deg;                    /* 渚挎嵎瀛楁锛氱浉瀵硅搴?deg銆?*/
    volatile float total_angle_deg;              /* 渚挎嵎瀛楁锛氳繛缁疮璁¤搴?deg銆?*/
    volatile float speed_dps;                    /* 渚挎嵎瀛楁锛氳閫熷害 deg/s銆?*/
} m3508_service_t;

/* LK 鐢垫満鏈嶅姟瀵硅薄銆傚綋鍓嶇敤浜庢嫧鐩橈紝鎸?RMD / LK 甯歌 CAN 鍗忚澶勭悊銆?*/
typedef struct
{
    CAN_HandleTypeDef *hcan;                     /* 鐢垫満鎵€鍦?CAN 鍙ユ焺銆?*/
    uint8_t motor_id;                            /* LK 鐢垫満 ID锛孲tdId 閫氬父涓?0x140 + ID銆?*/
    uint32_t std_id;                             /* LK 鐢垫満鏀跺彂浣跨敤鐨勬爣鍑?ID銆?*/
    uint8_t initialized;                         /* 鏄惁宸茬粡鐢ㄧ涓€甯у缓绔嬪崟鍦堢紪鐮佸櫒闆剁偣銆?*/
    uint16_t raw_ecd;                            /* 褰撳墠 16 bit 鍗曞湀缂栫爜鍣ㄥ€笺€?*/
    uint16_t last_raw_ecd;                       /* 涓婁竴甯?16 bit 鍗曞湀缂栫爜鍣ㄥ€笺€?*/
    uint16_t ecd_bias;                           /* 涓婄數鍚庣涓€甯х紪鐮佸櫒鍊硷紝浣滀负鐩稿闆剁偣銆?*/
    int16_t ecd_delta;                           /* 鏈抚鐩稿涓婁竴甯х殑鏈€鐭紪鐮佸櫒澧為噺銆?*/
    int32_t round_count;                         /* 鏍规嵁缂栫爜鍣ㄨ法闆剁疮璁″嚭鐨勫湀鏁般€?*/
    int32_t total_ecd;                           /* 浠庝笂鐢甸浂鐐瑰紑濮嬬疮璁＄殑杩炵画缂栫爜鍣ㄨ鏁般€?*/
    int16_t iq_output;                           /* 涓婂眰缂撳瓨鐨?LK iq/鍔涚煩鎺у埗杈撳嚭銆?*/
    volatile uint8_t online;                     /* 鏈€杩?MOTOR_ONLINE_TIMEOUT_MS 鍐呮槸鍚︽敹鍒板弽棣堛€?*/
    volatile uint8_t output_enabled;             /* 宸茬粡鍙戦€佽繃涓婄數/鍔涚煩鎺у埗锛屼笖鏈彂閫?stop銆?*/
    volatile uint32_t last_update_tick;          /* 鏈€杩戜竴娆℃敹鍒板弽棣堢殑 HAL tick銆?*/
    volatile uint8_t temperature;                /* LK 鐘舵€?2 鍙嶉娓╁害銆?*/
    volatile int16_t iq_feedback;                /* LK 鐘舵€?2 鍙嶉 iq/鍔涚煩鐢垫祦銆?*/
    volatile float speed_dps;                    /* LK 鐘舵€?2 鍙嶉閫熷害锛屽崟浣嶆寜鍗忚璁颁负 deg/s銆?*/
    volatile float angle_deg;                    /* 浠?ecd_bias 涓洪浂鐐圭殑鐩稿瑙掑害锛屽崟浣?deg銆?*/
    volatile float total_angle_deg;              /* 浠庝笂鐢靛紑濮嬬疮璁＄殑杩炵画瑙掑害锛屽崟浣?deg銆?*/
} lk_motor_service_t;

/*
 * 鍒濆鍖?motor 鏈嶅姟灞傘€? *
 * 浣滅敤锛? * - 娓呯┖ GM6020 / M3508 / LK 涓夌被鐢垫満鐨勬敞鍐岃〃锛? * - 涓嶇洿鎺ュ惎鍔?CAN锛孋AN 鐨勮繃婊ゅ櫒銆佸惎鍔ㄥ拰涓柇鐢?BSP_Init() 璐熻矗锛? * - 閫氬父鍦?BSP_Init() 涓€佷笂灞備换鍔℃敞鍐岀數鏈轰箣鍓嶈皟鐢ㄤ竴娆°€? */
void Motor_Init(void);

/*
 * 娉ㄥ唽涓€涓?GM6020 鐢垫満锛屽苟澹版槑瀹冨悗缁娇鐢ㄧ數娴佺幆鎺у埗甯с€? *
 * 鍙傛暟锛? * - motor锛氫笂灞備换鍔℃寔鏈夌殑 GM6020 鏈嶅姟瀵硅薄锛? * - hcan锛氱數鏈烘墍鍦?CAN锛屼緥濡?&hcan1 鎴?&hcan2锛? * - motor_id锛氱數鏈烘嫧鐮?ID锛岄€氬父涓?1~7銆? *
 * 娉ㄥ唽鍚庯紝Motor_ProcessCanMessage() 浼氭牴鎹?hcan + feedback_std_id 鑷姩鍒锋柊璇ュ璞°€? */
HAL_StatusTypeDef Motor_RegisterGm6020CurrentLoop(gm6020_service_t *motor,
                                                   CAN_HandleTypeDef *hcan,
                                                   uint8_t motor_id);

/*
 * 娉ㄥ唽涓€涓?GM6020 鐢垫満锛屽苟澹版槑瀹冨悗缁娇鐢ㄧ數鍘嬬幆鎺у埗甯с€? *
 * GM6020 鐢靛帇鐜娇鐢?0x1FF / 0x2FF 鎺у埗甯э紱鐢垫祦鐜娇鐢?0x1FE / 0x2FE銆? * 涓ょ妯″紡鐨勫弽棣堝抚 ID 鐩稿悓锛屽彧鏄彂閫佹帶鍒跺抚鐨勬爣鍑?ID 涓嶅悓銆? */
HAL_StatusTypeDef Motor_RegisterGm6020VoltageLoop(gm6020_service_t *motor,
                                                   CAN_HandleTypeDef *hcan,
                                                   uint8_t motor_id);

/*
 * 缂撳瓨 GM6020 鐢垫祦鐜緭鍑恒€? *
 * 杩欎釜鍑芥暟鍙敼 motor->output锛屼笉浼氱珛鍒诲彂 CAN銆? * 鐪熸鍙戝抚鐢?Motor_SendGm6020CurrentLoopFrame() 瀹屾垚锛屾柟渚夸笂灞傛妸鍚岀粍鐢垫満涓€娆℃墦鍖呭彂閫併€? */
void Motor_SetGm6020CurrentLoopOutput(gm6020_service_t *motor, int16_t output);

/*
 * 缂撳瓨 GM6020 鐢靛帇鐜緭鍑恒€? *
 * 鐢ㄦ硶涓?Motor_SetGm6020CurrentLoopOutput() 鐩稿悓锛屽彧鏄姹傜數鏈烘敞鍐屼负鐢靛帇鐜ā寮忋€? */
void Motor_SetGm6020VoltageLoopOutput(gm6020_service_t *motor, int16_t output);

/*
 * 鍙戦€佷竴甯?GM6020 鐢垫祦鐜帶鍒跺抚銆? *
 * 浼犲叆鐨勫璞″繀椤诲湪鍚屼竴璺?CAN銆佸悓涓€鎺у埗缁勶細
 * - ID1~ID4 浼氭墦鍖呭埌 0x1FE锛? * - ID5~ID7 浼氭墦鍖呭埌 0x2FE銆? *
 * 鍏佽浼?NULL锛岃〃绀鸿妲戒綅杈撳嚭 0锛涗絾鑷冲皯瑕佷紶鍏ヤ竴涓湁鏁堢數鏈哄璞°€? */
HAL_StatusTypeDef Motor_SendGm6020CurrentLoopFrame(const gm6020_service_t *motor1,
                                                   const gm6020_service_t *motor2,
                                                   const gm6020_service_t *motor3,
                                                   const gm6020_service_t *motor4);

/*
 * 鍙戦€佷竴甯?GM6020 鐢靛帇鐜帶鍒跺抚銆? *
 * 鍒嗙粍瑙勫垯锛? * - ID1~ID4 浼氭墦鍖呭埌 0x1FF锛? * - ID5~ID7 浼氭墦鍖呭埌 0x2FF銆? */
HAL_StatusTypeDef Motor_SendGm6020VoltageLoopFrame(const gm6020_service_t *motor1,
                                                   const gm6020_service_t *motor2,
                                                   const gm6020_service_t *motor3,
                                                   const gm6020_service_t *motor4);

/* 鍒锋柊骞惰繑鍥?GM6020 鍦ㄧ嚎鐘舵€併€傝秴杩?MOTOR_ONLINE_TIMEOUT_MS 鏈敹鍒板弽棣堝嵆璁や负绂荤嚎銆?*/
uint8_t Motor_Gm6020IsOnline(const gm6020_service_t *motor);

/*
 * 娉ㄥ唽涓€涓?M3508 / M2006 鐢垫満锛屽苟澹版槑瀹冧娇鐢?C620 / C610 鐢垫祦鐜帶鍒跺抚銆? *
 * 鍙傛暟 motor_id 涓虹數鏈烘嫧鐮?ID锛岄€氬父涓?1~8锛? * - ID1~ID4 鐨勬帶鍒跺抚涓?0x200锛? * - ID5~ID8 鐨勬帶鍒跺抚涓?0x1FF銆? */
HAL_StatusTypeDef Motor_RegisterM3508CurrentLoop(m3508_service_t *motor,
                                                  CAN_HandleTypeDef *hcan,
                                                  uint8_t motor_id);

/*
 * 缂撳瓨 M3508 / M2006 鐢垫祦鐜緭鍑恒€? *
 * 鍙慨鏀规湇鍔″璞￠噷鐨?output 瀛楁锛屼笉绔嬪嵆鍙戦€?CAN銆? */
void Motor_SetM3508CurrentLoopOutput(m3508_service_t *motor, int16_t output);

/*
 * 鍙戦€佷竴甯?M3508 / M2006 鐢垫祦鐜帶鍒跺抚銆? *
 * 浼犲叆鐨勫璞″繀椤诲湪鍚屼竴璺?CAN銆佸悓涓€鎺у埗缁勩€? * NULL 琛ㄧず瀵瑰簲妲戒綅杈撳嚭 0銆? */
HAL_StatusTypeDef Motor_SendM3508CurrentLoopFrame(const m3508_service_t *motor1,
                                                  const m3508_service_t *motor2,
                                                  const m3508_service_t *motor3,
                                                  const m3508_service_t *motor4);

/* 鍒锋柊骞惰繑鍥?M3508 / M2006 鍦ㄧ嚎鐘舵€併€?*/
uint8_t Motor_M3508IsOnline(const m3508_service_t *motor);

/*
 * 娉ㄥ唽涓€涓?LK / RMD 鍗忚鐢垫満銆? *
 * LK 鐨勬敹鍙戞爣鍑嗗抚 ID 閫氬父涓?0x140 + motor_id銆? * 褰撳墠宸ョ▼閲屾嫧鐩樼敤 CAN2 ID5锛屽嵆鏍囧噯甯?ID 0x145銆? */
HAL_StatusTypeDef Motor_RegisterLk(lk_motor_service_t *motor,
                                    CAN_HandleTypeDef *hcan,
                                    uint8_t motor_id);

/*
 * 缂撳瓨 LK 鐢垫満 iq/鍔涚煩杈撳嚭銆? *
 * 杩欎釜鍑芥暟鍙啓 iq_output锛屼笉鍙?CAN锛涘彂甯т娇鐢?Motor_LkSendIqControl()銆? */
void Motor_SetLkIqOutput(lk_motor_service_t *motor, int16_t output);

/* 鍙戦€?LK 涓婄數 / 杈撳嚭浣胯兘鍛戒护 0x88銆?*/
HAL_StatusTypeDef Motor_LkSendPowerOn(lk_motor_service_t *motor);

/* 鍙戦€?LK 鍋滄鍛戒护 0x81锛屽苟鍦ㄥ彂閫佹垚鍔熷悗娓?output_enabled 鏍囧織銆?*/
HAL_StatusTypeDef Motor_LkSendStop(lk_motor_service_t *motor);

/*
 * 鍙戦€?LK 鐘舵€?2 鏌ヨ鍛戒护 0x9C銆? *
 * 鐘舵€?2 鍥炲寘閲屽寘鍚俯搴︺€乮q 鍙嶉銆侀€熷害鍜岀紪鐮佸櫒鍊硷紝
 * 褰撳墠鎷ㄧ洏鍦ㄧ嚎妫€娴嬪拰鍙嶉鍒锋柊涓昏渚濊禆杩欐潯鍛戒护銆? */
HAL_StatusTypeDef Motor_LkSendReadState2Request(lk_motor_service_t *motor);

/* 鍙戦€?LK iq/鍔涚煩鎺у埗鍛戒护 0xA1锛屽懡浠ゅ€兼潵鑷?motor->iq_output銆?*/
HAL_StatusTypeDef Motor_LkSendIqControl(lk_motor_service_t *motor);

/* 鍒锋柊骞惰繑鍥?LK 鐢垫満鍦ㄧ嚎鐘舵€併€?*/
uint8_t Motor_LkIsOnline(const lk_motor_service_t *motor);

/*
 * CAN 鎺ユ敹甯х粺涓€鍒嗗彂鍏ュ彛銆? *
 * BSP 鍦?HAL_CAN_RxFifo0MsgPendingCallback() 涓彇鍑?CAN 甯у悗璋冪敤姝ゅ嚱鏁般€? * motor 灞備細鎸?hcan + StdId 鍖归厤娉ㄥ唽琛紝鎵惧埌瀵瑰簲瀵硅薄鍚庡埛鏂板弽棣堝拰鍦ㄧ嚎鏃堕棿鎴炽€? */
void Motor_ProcessCanMessage(CAN_HandleTypeDef *hcan,
                             const CAN_RxHeaderTypeDef *rx_header,
                             const uint8_t rx_data[8]);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
