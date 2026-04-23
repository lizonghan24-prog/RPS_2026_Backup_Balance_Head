#include "motor.h"

#include <string.h>


/*
 * 鐢垫満鏈嶅姟灞傝亴璐ｈ鏄庯細
 * 1. 涓婂眰浠诲姟娉ㄥ唽鈥滄煇涓數鏈哄璞″湪鏌愪竴璺?CAN銆佹煇涓?ID 涓娾€濓紱
 * 2. BSP 鏀跺埌 CAN 甯у悗璋冪敤 Motor_ProcessCanMessage()锛? * 3. 鏈枃浠舵寜 hcan + StdId 鎵惧埌瀵硅薄锛屽埛鏂板弽棣堛€佸湪绾挎爣蹇楀拰鏃堕棿鎴筹紱
 * 4. 涓婂眰浠诲姟鍐?output 鍚庯紝璋冪敤鍙戦€佸嚱鏁版妸鍚岀粍鐢垫満涓€娆℃€ф墦鍖呭彂鍑恒€? *
 * 娉ㄦ剰锛氭湰鏂囦欢涓嶈礋璐?CAN 澶栬鍒濆鍖栵紝涔熶笉璐熻矗闂幆 PID 璁＄畻銆? */

/*
 * DJI 甯哥敤鏍囧噯甯?ID锛? * - M3508/M2006 鍙嶉锛?x201 ~ 0x208锛屽搴旂數鏈?ID1 ~ ID8锛? * - M3508/M2006 鎺у埗锛欼D1~ID4 鍙?0x200锛孖D5~ID8 鍙?0x1FF锛? * - GM6020 鍙嶉锛?x205 ~ 0x20B锛屽搴旂數鏈?ID1 ~ ID7锛? * - GM6020 鐢垫祦鎺у埗锛欼D1~ID4 鍙?0x1FE锛孖D5~ID7 鍙?0x2FE锛? * - GM6020 鐢靛帇鎺у埗锛欼D1~ID4 鍙?0x1FF锛孖D5~ID7 鍙?0x2FF銆? */
#define MOTOR_DJI_M3508_FEEDBACK_BASE           0x200U
#define MOTOR_DJI_GM6020_FEEDBACK_BASE          0x204U
#define MOTOR_DJI_M3508_CMD_LOW                 0x200U
#define MOTOR_DJI_M3508_CMD_HIGH                0x1FFU
#define MOTOR_DJI_GM6020_CURRENT_CMD_LOW        0x1FEU
#define MOTOR_DJI_GM6020_CURRENT_CMD_HIGH       0x2FEU
#define MOTOR_DJI_GM6020_VOLTAGE_CMD_LOW        0x1FFU
#define MOTOR_DJI_GM6020_VOLTAGE_CMD_HIGH       0x2FFU

/*
 * LK / RMD 甯哥敤鏍囧噯甯?ID 鍜屽懡浠ゅ瓧銆? * 鐢垫満 ID5 瀵瑰簲 StdId 0x145锛屼篃灏辨槸 0x140 + 5銆? */
#define MOTOR_LK_STD_ID_BASE                    0x140U
#define MOTOR_LK_CMD_STOP                       0x81U
#define MOTOR_LK_CMD_POWER_ON                   0x88U
#define MOTOR_LK_CMD_READ_STATE_2               0x9CU
#define MOTOR_LK_CMD_IQ_CONTROL                 0xA1U

#define MOTOR_MAX_GM6020_COUNT                  8U      /* GM6020 娉ㄥ唽琛ㄥ閲忥紝瓒冲瑕嗙洊涓€鏉?CAN 涓婄殑甯哥敤 ID銆?*/
#define MOTOR_MAX_M3508_COUNT                   8U      /* M3508/M2006 娉ㄥ唽琛ㄥ閲忥紝瀵瑰簲 DJI ID1~ID8銆?*/
#define MOTOR_MAX_LK_COUNT                      8U      /* LK 娉ㄥ唽琛ㄥ閲忥紝褰撳墠鎷ㄧ洏鍙敤 1 涓紝棰勭暀鎵╁睍銆?*/

static gm6020_service_t *gm6020_registry[MOTOR_MAX_GM6020_COUNT];
static m3508_service_t *m3508_registry[MOTOR_MAX_M3508_COUNT];
static lk_motor_service_t *lk_registry[MOTOR_MAX_LK_COUNT];

/*
 * 娉ㄥ唽琛ㄥ彧淇濆瓨瀵硅薄鎸囬拡锛屼笉鐢宠涔熶笉閲婃斁鍐呭瓨銆? * 鍥犳浼犺繘 Register 鍑芥暟鐨勫璞″繀椤绘槸鍏ㄥ眬銆侀潤鎬佹垨闀挎湡瀛樺湪鐨勪换鍔′笂涓嬫枃鎴愬憳銆? */

/* 鎶婂ぇ绔牸寮忕殑涓や釜瀛楄妭杩樺師鎴?int16_t锛孌JI 鍙嶉鍜屾帶鍒跺抚閮戒娇鐢ㄨ繖绉嶉『搴忋€?*/
static int16_t Motor_ReadInt16BE(const uint8_t *data)
{
    return (int16_t)((uint16_t)((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

/* 鎶婂皬绔牸寮忕殑涓や釜瀛楄妭杩樺師鎴?int16_t锛孡K 鐘舵€佸抚浣跨敤杩欑椤哄簭銆?*/
static int16_t Motor_ReadInt16LE(const uint8_t *data)
{
    return (int16_t)((uint16_t)((uint16_t)data[1] << 8) | (uint16_t)data[0]);
}

/* 鎶婂皬绔牸寮忕殑涓や釜瀛楄妭杩樺師鎴?uint16_t锛岀敤鏉ヨВ鏋?LK 鍗曞湀缂栫爜鍣ㄥ€笺€?*/
static uint16_t Motor_ReadUint16LE(const uint8_t *data)
{
    return (uint16_t)((uint16_t)((uint16_t)data[1] << 8) | (uint16_t)data[0]);
}

/* 鎶?int16_t 鎷嗘垚楂樺瓧鑺傚湪鍓嶃€佷綆瀛楄妭鍦ㄥ悗鐨勬牸寮忋€?*/
static void Motor_WriteInt16BE(uint8_t *data, uint8_t slot, int16_t value)
{
    uint8_t index;

    index = (uint8_t)(slot * 2U);
    data[index] = (uint8_t)((uint16_t)value >> 8);
    data[index + 1U] = (uint8_t)((uint16_t)value & 0xFFU);
}

/* LK 鎺у埗鍛戒护閲?iq 缁欏畾鍊兼斁鍦?data[4]/data[5]锛岄噰鐢ㄥ皬绔牸寮忋€?*/
static void Motor_WriteInt16LE(uint8_t *data, uint8_t index, int16_t value)
{
    data[index] = (uint8_t)((uint16_t)value & 0xFFU);
    data[index + 1U] = (uint8_t)((uint16_t)value >> 8);
}

/* HAL CAN 鐨勬爣鍑嗘暟鎹抚鍙戦€佸皝瑁咃紝鎵€鏈?motor 灞傚彂閫佹渶缁堥兘璧拌繖閲屻€?*/
static HAL_StatusTypeDef Motor_SendStdFrame(CAN_HandleTypeDef *hcan,
                                            uint32_t std_id,
                                            const uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    if ((hcan == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    memset(&tx_header, 0, sizeof(tx_header));
    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
}


/*
 * 鏃ф爣鍑嗗簱鎺ュ彛浼犲叆鐨勬槸 CAN1 / CAN2 澶栬鍩哄湴鍧€銆? * HAL 鍙戦€佸嚱鏁伴渶瑕?CAN_HandleTypeDef锛岃繖閲屾妸澶栬鍩哄湴鍧€鏄犲皠鍥?CubeMX 鐢熸垚鐨勫彞鏌勩€? */

/*
 * 娉ㄥ唽琛ㄤ笉鏄寜 ID 鐩存帴涓嬫爣璁块棶锛岃€屾槸鎸夆€淐AN 鍙ユ焺 + StdId鈥濆尮閰嶃€? * 杩欐牱鍚屼竴涓數鏈?ID 鍙互鍚屾椂鍑虹幇鍦?CAN1 鍜?CAN2 涓婏紝浜掍笉鍐茬獊銆? */
static HAL_StatusTypeDef Motor_RegisterGm6020Object(gm6020_service_t *motor)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_MAX_GM6020_COUNT; ++i)
    {
        if ((gm6020_registry[i] == motor)
         || ((gm6020_registry[i] != NULL)
          && (gm6020_registry[i]->hcan == motor->hcan)
          && (gm6020_registry[i]->feedback_std_id == motor->feedback_std_id)))
        {
            gm6020_registry[i] = motor;
            return HAL_OK;
        }
    }

    for (i = 0U; i < MOTOR_MAX_GM6020_COUNT; ++i)
    {
        if (gm6020_registry[i] == NULL)
        {
            gm6020_registry[i] = motor;
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}

/*
 * 鎶?M3508/M2006 瀵硅薄鎸傚叆娉ㄥ唽琛ㄣ€? * 鍏佽閲嶅娉ㄥ唽鍚屼竴涓璞★紝鎴栫敤鏂板璞¤鐩栧悓涓€璺?CAN + 鍚屼竴鍙嶉 ID 鐨勬棫鏉＄洰銆? */
static HAL_StatusTypeDef Motor_RegisterM3508Object(m3508_service_t *motor)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_MAX_M3508_COUNT; ++i)
    {
        if ((m3508_registry[i] == motor)
         || ((m3508_registry[i] != NULL)
          && (m3508_registry[i]->hcan == motor->hcan)
          && (m3508_registry[i]->feedback_std_id == motor->feedback_std_id)))
        {
            m3508_registry[i] = motor;
            return HAL_OK;
        }
    }

    for (i = 0U; i < MOTOR_MAX_M3508_COUNT; ++i)
    {
        if (m3508_registry[i] == NULL)
        {
            m3508_registry[i] = motor;
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}

/*
 * 鎶?LK 瀵硅薄鎸傚叆娉ㄥ唽琛ㄣ€? * LK 鍗忚鏀跺彂鍏辩敤 0x140 + ID锛屾墍浠ュ尮閰嶉敭浣跨敤 hcan + std_id銆? */
static HAL_StatusTypeDef Motor_RegisterLkObject(lk_motor_service_t *motor)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_MAX_LK_COUNT; ++i)
    {
        if ((lk_registry[i] == motor)
         || ((lk_registry[i] != NULL)
          && (lk_registry[i]->hcan == motor->hcan)
          && (lk_registry[i]->std_id == motor->std_id)))
        {
            lk_registry[i] = motor;
            return HAL_OK;
        }
    }

    for (i = 0U; i < MOTOR_MAX_LK_COUNT; ++i)
    {
        if (lk_registry[i] == NULL)
        {
            lk_registry[i] = motor;
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}

/* 鍒锋柊鍦ㄧ嚎鏃堕棿鎴炽€傝繖涓嚱鏁板彧璇存槑鈥滄敹鍒拌繃鍚堟硶甯р€濓紝鍏蜂綋瑙ｆ瀽鐢卞灞傚畬鎴愩€?*/
static void Motor_TouchOnline(volatile uint8_t *online, volatile uint32_t *last_update_tick)
{
    *last_update_tick = HAL_GetTick();
    *online = 1U;
}

/*
 * 鍒ゆ柇鏌愪釜鏃堕棿鎴虫槸鍚︿粛鐒舵湁鏁堛€? * HAL_GetTick() 鏄?uint32_t锛岀洿鎺ュ仛鏃犵鍙峰噺娉曞彲浠ヨ嚜鐒跺鐞?tick 鍥炵粫銆? */
static uint8_t Motor_IsRecent(uint32_t last_update_tick)
{
    return (uint8_t)((HAL_GetTick() - last_update_tick) <= MOTOR_ONLINE_TIMEOUT_MS);
}

/*
 * DJI 缂栫爜鍣ㄨ法闆跺鐞嗭細
 * - 鍘熷鍊间粠 8191 璺冲埌 0 闄勮繎锛岃鏄庢鍚戣法闆讹紝瀹為檯澧為噺瑕佸姞 8192锛? * - 鍘熷鍊间粠 0 璺冲埌 8191 闄勮繎锛岃鏄庡弽鍚戣法闆讹紝瀹為檯澧為噺瑕佸噺 8192銆? *
 * 杩欐牱鍙互鎶婂崟鍦堢紪鐮佸櫒鍙樻垚杩炵画瑙掑害锛屾柟渚夸笂灞傚仛浣嶇疆闂幆鎴栬皟璇曡瀵熴€? */
static void Motor_UpdateDjiFeedback(dji_motor_feedback_t *feedback, const uint8_t data[8])
{
    uint16_t raw_ecd;
    int16_t delta;

    raw_ecd = (uint16_t)Motor_ReadInt16BE(&data[0]);

    if (feedback->initialized == 0U)
    {
        feedback->initialized = 1U;
        feedback->raw_ecd = raw_ecd;
        feedback->last_raw_ecd = raw_ecd;
        feedback->ecd_bias = raw_ecd;
        feedback->ecd_delta = 0;
        feedback->round_count = 0;
        feedback->total_ecd = 0;
    }
    else
    {
        feedback->last_raw_ecd = feedback->raw_ecd;
        feedback->raw_ecd = raw_ecd;

        delta = (int16_t)((int32_t)feedback->raw_ecd - (int32_t)feedback->last_raw_ecd);
        if (delta < -MOTOR_DJI_ENCODER_HALF_RANGE)
        {
            delta = (int16_t)(delta + (int16_t)MOTOR_DJI_ENCODER_RANGE);
            feedback->round_count++;
        }
        else if (delta > MOTOR_DJI_ENCODER_HALF_RANGE)
        {
            delta = (int16_t)(delta - (int16_t)MOTOR_DJI_ENCODER_RANGE);
            feedback->round_count--;
        }

        feedback->ecd_delta = delta;
        feedback->total_ecd += delta;
    }

    feedback->speed_rpm = Motor_ReadInt16BE(&data[2]);
    feedback->given_current = Motor_ReadInt16BE(&data[4]);
    feedback->temperature = data[6];
    feedback->angle_deg = ((float)((int32_t)feedback->raw_ecd - (int32_t)feedback->ecd_bias)
                         * MOTOR_DJI_ENCODER_DEG_PER_TICK)
                        + ((float)feedback->round_count * 360.0f);
    feedback->total_angle_deg = (float)feedback->total_ecd * MOTOR_DJI_ENCODER_DEG_PER_TICK;
    feedback->speed_dps = (float)feedback->speed_rpm * 6.0f;
}

/* 鎶婇€氱敤 DJI 鍙嶉鍚屾鍒?GM6020 渚挎嵎瀛楁锛屽噺灏戜笂灞傜洿鎺ヨ闂?feedback 瀛愮粨鏋勭殑楹荤儲銆?*/
static void Motor_CopyDjiFeedbackToGm6020(gm6020_service_t *motor)
{
    motor->speed_rpm = motor->feedback.speed_rpm;
    motor->given_current = motor->feedback.given_current;
    motor->temperature = motor->feedback.temperature;
    motor->angle_deg = motor->feedback.angle_deg;
    motor->total_angle_deg = motor->feedback.total_angle_deg;
    motor->speed_dps = motor->feedback.speed_dps;
}

/* 鎶婇€氱敤 DJI 鍙嶉鍚屾鍒?M3508 渚挎嵎瀛楁銆?*/
static void Motor_CopyDjiFeedbackToM3508(m3508_service_t *motor)
{
    motor->speed_rpm = motor->feedback.speed_rpm;
    motor->given_current = motor->feedback.given_current;
    motor->temperature = motor->feedback.temperature;
    motor->angle_deg = motor->feedback.angle_deg;
    motor->total_angle_deg = motor->feedback.total_angle_deg;
    motor->speed_dps = motor->feedback.speed_dps;
}

/*
 * LK 鐘舵€?2 鍙嶉瑙ｆ瀽锛? * data[0] = 0x9C 鎴栧姏鐭╂帶鍒惰繑鍥炵殑 0xA1锛? * data[1] = 娓╁害锛? * data[2..3] = iq/鍔涚煩鐢垫祦锛屽皬绔紱
 * data[4..5] = 閫熷害锛屽皬绔紱
 * data[6..7] = 鍗曞湀缂栫爜鍣紝灏忕銆? */
static void Motor_UpdateLkState2(lk_motor_service_t *motor, const uint8_t data[8])
{
    uint16_t raw_ecd;
    int32_t delta;

    raw_ecd = Motor_ReadUint16LE(&data[6]);

    if (motor->initialized == 0U)
    {
        motor->initialized = 1U;
        motor->raw_ecd = raw_ecd;
        motor->last_raw_ecd = raw_ecd;
        motor->ecd_bias = raw_ecd;
        motor->ecd_delta = 0;
        motor->round_count = 0;
        motor->total_ecd = 0;
    }
    else
    {
        motor->last_raw_ecd = motor->raw_ecd;
        motor->raw_ecd = raw_ecd;

        delta = (int32_t)motor->raw_ecd - (int32_t)motor->last_raw_ecd;
        if (delta < -MOTOR_LK_ENCODER_HALF_RANGE)
        {
            delta += (int32_t)MOTOR_LK_ENCODER_RANGE;
            motor->round_count++;
        }
        else if (delta > MOTOR_LK_ENCODER_HALF_RANGE)
        {
            delta -= (int32_t)MOTOR_LK_ENCODER_RANGE;
            motor->round_count--;
        }

        motor->ecd_delta = (int16_t)delta;
        motor->total_ecd += delta;
    }

    motor->temperature = data[1];
    motor->iq_feedback = Motor_ReadInt16LE(&data[2]);
    motor->speed_dps = (float)Motor_ReadInt16LE(&data[4]);
    motor->angle_deg = ((float)((int32_t)motor->raw_ecd - (int32_t)motor->ecd_bias)
                      * MOTOR_LK_ENCODER_DEG_PER_TICK)
                     + ((float)motor->round_count * 360.0f);
    motor->total_angle_deg = (float)motor->total_ecd * MOTOR_LK_ENCODER_DEG_PER_TICK;
}

/* DJI 鍥涚數鏈烘帶鍒跺抚鐨勫師濮嬪彂閫佸嚱鏁帮紝鏃?Set_* 鍖呰涔熶細澶嶇敤瀹冦€?*/
/*
 * 鎸夌數鏈?ID 鑷姩鍐冲畾 DJI 鎺у埗甯у簲璇ヨ蛋浣庣粍杩樻槸楂樼粍銆? * 浼犲叆鐨勫洓涓寚閽堜笉鏄己鍒朵唬琛?slot1~slot4锛岃€屾槸鏍规嵁 motor_id 鏀惧埌姝ｇ‘瀛楄妭浣嶃€? */
static HAL_StatusTypeDef Motor_SendGm6020FrameByMode(gm6020_control_mode_t mode,
                                                     uint32_t low_std_id,
                                                     uint32_t high_std_id,
                                                     const gm6020_service_t *motor1,
                                                     const gm6020_service_t *motor2,
                                                     const gm6020_service_t *motor3,
                                                     const gm6020_service_t *motor4)
{
    const gm6020_service_t *motors[4];
    CAN_HandleTypeDef *hcan;
    uint8_t tx_data[8];
    uint8_t i;
    uint8_t high_group;
    uint8_t have_motor;
    uint8_t slot;
    uint32_t std_id;

    motors[0] = motor1;
    motors[1] = motor2;
    motors[2] = motor3;
    motors[3] = motor4;
    hcan = NULL;
    high_group = 0U;
    have_motor = 0U;
    memset(tx_data, 0, sizeof(tx_data));

    for (i = 0U; i < 4U; ++i)
    {
        /* NULL 琛ㄧず璇ユ帶鍒舵Ы浣嶈緭鍑?0銆?*/
        if (motors[i] == NULL)
        {
            continue;
        }

        /* 淇濇姢妫€鏌ワ細瀵硅薄蹇呴』鏈夋晥銆両D 鑼冨洿姝ｇ‘銆佹帶鍒舵ā寮忓尮閰嶃€?*/
        if ((motors[i]->hcan == NULL)
         || (motors[i]->motor_id < 1U)
         || (motors[i]->motor_id > 8U)
         || (motors[i]->control_mode != mode))
        {
            return HAL_ERROR;
        }

        if (hcan == NULL)
        {
            /* 绗竴鍙版湁鏁堢數鏈哄喅瀹氭湰甯т娇鐢ㄥ摢涓€璺?CAN銆佷綆缁勮繕鏄珮缁勩€?*/
            hcan = motors[i]->hcan;
            high_group = (uint8_t)((motors[i]->motor_id >= 5U) ? 1U : 0U);
        }
        else if ((hcan != motors[i]->hcan)
              || (high_group != (uint8_t)((motors[i]->motor_id >= 5U) ? 1U : 0U)))
        {
            /* 鍚屼竴甯т笉鑳芥贩鍙戜笉鍚?CAN 鎴栦笉鍚?ID 鍒嗙粍銆?*/
            return HAL_ERROR;
        }

        /* ID1/5 鏀?slot0锛孖D2/6 鏀?slot1锛屼互姝ょ被鎺ㄣ€?*/
        slot = (uint8_t)((motors[i]->motor_id - 1U) % 4U);
        Motor_WriteInt16BE(tx_data, slot, motors[i]->output);
        have_motor = 1U;
    }

    if (have_motor == 0U)
    {
        return HAL_ERROR;
    }

    std_id = (high_group != 0U) ? high_std_id : low_std_id;
    return Motor_SendStdFrame(hcan, std_id, tx_data);
}

/* M3508/C620/C610 鐨勫垎缁勮鍒欎笌 GM6020 绫讳技锛屼絾鎺у埗甯?ID 涓嶅悓銆?*/
static HAL_StatusTypeDef Motor_SendM3508FrameById(const m3508_service_t *motor1,
                                                  const m3508_service_t *motor2,
                                                  const m3508_service_t *motor3,
                                                  const m3508_service_t *motor4)
{
    const m3508_service_t *motors[4];
    CAN_HandleTypeDef *hcan;
    uint8_t tx_data[8];
    uint8_t i;
    uint8_t high_group;
    uint8_t have_motor;
    uint8_t slot;
    uint32_t std_id;

    motors[0] = motor1;
    motors[1] = motor2;
    motors[2] = motor3;
    motors[3] = motor4;
    hcan = NULL;
    high_group = 0U;
    have_motor = 0U;
    memset(tx_data, 0, sizeof(tx_data));

    for (i = 0U; i < 4U; ++i)
    {
        /* NULL 琛ㄧず璇ユ帶鍒舵Ы浣嶈緭鍑?0銆?*/
        if (motors[i] == NULL)
        {
            continue;
        }

        /* M3508/M2006 鐨勬湁鏁?ID 鑼冨洿鏄?1~8銆?*/
        if ((motors[i]->hcan == NULL)
         || (motors[i]->motor_id < 1U)
         || (motors[i]->motor_id > 8U))
        {
            return HAL_ERROR;
        }

        if (hcan == NULL)
        {
            /* 绗竴鍙版湁鏁堢數鏈哄喅瀹氭湰甯т娇鐢ㄥ摢涓€璺?CAN銆佷綆缁勮繕鏄珮缁勩€?*/
            hcan = motors[i]->hcan;
            high_group = (uint8_t)((motors[i]->motor_id >= 5U) ? 1U : 0U);
        }
        else if ((hcan != motors[i]->hcan)
              || (high_group != (uint8_t)((motors[i]->motor_id >= 5U) ? 1U : 0U)))
        {
            /* 鍚屼竴甯т笉鑳芥贩鍙戜笉鍚?CAN 鎴栦笉鍚?ID 鍒嗙粍銆?*/
            return HAL_ERROR;
        }

        /* ID1/5 鏀?slot0锛孖D2/6 鏀?slot1锛屼互姝ょ被鎺ㄣ€?*/
        slot = (uint8_t)((motors[i]->motor_id - 1U) % 4U);
        Motor_WriteInt16BE(tx_data, slot, motors[i]->output);
        have_motor = 1U;
    }

    if (have_motor == 0U)
    {
        return HAL_ERROR;
    }

    std_id = (high_group != 0U) ? MOTOR_DJI_M3508_CMD_HIGH : MOTOR_DJI_M3508_CMD_LOW;
    return Motor_SendStdFrame(hcan, std_id, tx_data);
}

/*
 * 娓呯┖涓夌被鐢垫満娉ㄥ唽琛ㄣ€? * 璇ュ嚱鏁伴€氬父鍦?BSP 鍒濆鍖栭樁娈佃皟鐢ㄤ竴娆★紝涔嬪悗鐢辨帶鍒朵换鍔?鍙戝皠浠诲姟閲嶆柊娉ㄥ唽鐢垫満銆? */
void Motor_Init(void)
{
    memset(gm6020_registry, 0, sizeof(gm6020_registry));
    memset(m3508_registry, 0, sizeof(m3508_registry));
    memset(lk_registry, 0, sizeof(lk_registry));
}

/*
 * 娉ㄥ唽 GM6020 鐢垫祦鐜璞°€? * ID1 鐨勫弽棣堝抚鏄?0x205锛屽悗缁?ID 渚濇閫掑銆? */
HAL_StatusTypeDef Motor_RegisterGm6020CurrentLoop(gm6020_service_t *motor,
                                                   CAN_HandleTypeDef *hcan,
                                                   uint8_t motor_id)
{
    if ((motor == NULL) || (hcan == NULL) || (motor_id < 1U) || (motor_id > 8U))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->feedback_std_id = MOTOR_DJI_GM6020_FEEDBACK_BASE + (uint32_t)motor_id;
    motor->control_mode = MOTOR_GM6020_CONTROL_CURRENT;

    return Motor_RegisterGm6020Object(motor);
}

/*
 * 娉ㄥ唽 GM6020 鐢靛帇鐜璞°€? * 鍙嶉瑙ｆ瀽涓庣數娴佺幆涓€鑷达紝鍖哄埆鍙湪鍙戦€佹椂浣跨敤鐢靛帇鎺у埗甯?ID銆? */
HAL_StatusTypeDef Motor_RegisterGm6020VoltageLoop(gm6020_service_t *motor,
                                                   CAN_HandleTypeDef *hcan,
                                                   uint8_t motor_id)
{
    if ((motor == NULL) || (hcan == NULL) || (motor_id < 1U) || (motor_id > 8U))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->feedback_std_id = MOTOR_DJI_GM6020_FEEDBACK_BASE + (uint32_t)motor_id;
    motor->control_mode = MOTOR_GM6020_CONTROL_VOLTAGE;

    return Motor_RegisterGm6020Object(motor);
}

/* 缂撳瓨 GM6020 鐢垫祦鐜緭鍑猴紝涓嶇珛鍗冲彂閫?CAN銆?*/
void Motor_SetGm6020CurrentLoopOutput(gm6020_service_t *motor, int16_t output)
{
    if ((motor != NULL) && (motor->control_mode == MOTOR_GM6020_CONTROL_CURRENT))
    {
        motor->output = output;
    }
}

/* 缂撳瓨 GM6020 鐢靛帇鐜緭鍑猴紝涓嶇珛鍗冲彂閫?CAN銆?*/
void Motor_SetGm6020VoltageLoopOutput(gm6020_service_t *motor, int16_t output)
{
    if ((motor != NULL) && (motor->control_mode == MOTOR_GM6020_CONTROL_VOLTAGE))
    {
        motor->output = output;
    }
}

/*
 * 鍙戦€?GM6020 鐢垫祦鐜帶鍒跺抚銆? * 浼犲叆鐨勭數鏈哄繀椤诲睘浜庡悓涓€璺?CAN 鍜屽悓涓€ ID 鍒嗙粍銆? */
HAL_StatusTypeDef Motor_SendGm6020CurrentLoopFrame(const gm6020_service_t *motor1,
                                                   const gm6020_service_t *motor2,
                                                   const gm6020_service_t *motor3,
                                                   const gm6020_service_t *motor4)
{
    return Motor_SendGm6020FrameByMode(MOTOR_GM6020_CONTROL_CURRENT,
                                       MOTOR_DJI_GM6020_CURRENT_CMD_LOW,
                                       MOTOR_DJI_GM6020_CURRENT_CMD_HIGH,
                                       motor1,
                                       motor2,
                                       motor3,
                                       motor4);
}

/*
 * 鍙戦€?GM6020 鐢靛帇鐜帶鍒跺抚銆? * 鍒嗙粍瑙勫垯鍜岀數娴佺幆鐩稿悓锛屽彧鏄帶鍒?StdId 涓嶅悓銆? */
HAL_StatusTypeDef Motor_SendGm6020VoltageLoopFrame(const gm6020_service_t *motor1,
                                                   const gm6020_service_t *motor2,
                                                   const gm6020_service_t *motor3,
                                                   const gm6020_service_t *motor4)
{
    return Motor_SendGm6020FrameByMode(MOTOR_GM6020_CONTROL_VOLTAGE,
                                       MOTOR_DJI_GM6020_VOLTAGE_CMD_LOW,
                                       MOTOR_DJI_GM6020_VOLTAGE_CMD_HIGH,
                                       motor1,
                                       motor2,
                                       motor3,
                                       motor4);
}

/* 杩斿洖 GM6020 鍦ㄧ嚎鐘舵€侊紝瓒呰繃瓒呮椂鏃堕棿鏈埛鏂板垯杩斿洖 0銆?*/
uint8_t Motor_Gm6020IsOnline(const gm6020_service_t *motor)
{
    if ((motor == NULL) || (motor->online == 0U))
    {
        return 0U;
    }

    return Motor_IsRecent(motor->last_update_tick);
}

/*
 * 娉ㄥ唽 M3508/M2006 鐢垫祦鐜璞°€? * ID1 鐨勫弽棣堝抚鏄?0x201锛孖D8 鐨勫弽棣堝抚鏄?0x208銆? */
HAL_StatusTypeDef Motor_RegisterM3508CurrentLoop(m3508_service_t *motor,
                                                  CAN_HandleTypeDef *hcan,
                                                  uint8_t motor_id)
{
    if ((motor == NULL) || (hcan == NULL) || (motor_id < 1U) || (motor_id > 8U))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->feedback_std_id = MOTOR_DJI_M3508_FEEDBACK_BASE + (uint32_t)motor_id;

    return Motor_RegisterM3508Object(motor);
}

/* 缂撳瓨 M3508/M2006 鐢垫祦鐜緭鍑猴紝涓嶇珛鍗冲彂閫?CAN銆?*/
void Motor_SetM3508CurrentLoopOutput(m3508_service_t *motor, int16_t output)
{
    if (motor != NULL)
    {
        motor->output = output;
    }
}

/* 鍙戦€佷竴甯?M3508/M2006 鐢垫祦鐜帶鍒跺抚銆?*/
HAL_StatusTypeDef Motor_SendM3508CurrentLoopFrame(const m3508_service_t *motor1,
                                                  const m3508_service_t *motor2,
                                                  const m3508_service_t *motor3,
                                                  const m3508_service_t *motor4)
{
    return Motor_SendM3508FrameById(motor1, motor2, motor3, motor4);
}

/* 杩斿洖 M3508/M2006 鍦ㄧ嚎鐘舵€併€?*/
uint8_t Motor_M3508IsOnline(const m3508_service_t *motor)
{
    if ((motor == NULL) || (motor->online == 0U))
    {
        return 0U;
    }

    return Motor_IsRecent(motor->last_update_tick);
}

/*
 * 娉ㄥ唽 LK/RMD 鐢垫満銆? * 鏍囧噯甯?ID = 0x140 + motor_id锛屼緥濡?ID5 瀵瑰簲 0x145銆? */
HAL_StatusTypeDef Motor_RegisterLk(lk_motor_service_t *motor,
                                    CAN_HandleTypeDef *hcan,
                                    uint8_t motor_id)
{
    if ((motor == NULL) || (hcan == NULL) || (motor_id == 0U) || (motor_id > 0x7FU))
    {
        return HAL_ERROR;
    }

    memset(motor, 0, sizeof(*motor));
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->std_id = MOTOR_LK_STD_ID_BASE + (uint32_t)motor_id;

    return Motor_RegisterLkObject(motor);
}

/* 缂撳瓨 LK iq/鍔涚煩杈撳嚭锛屼笉绔嬪嵆鍙戦€?CAN銆?*/
void Motor_SetLkIqOutput(lk_motor_service_t *motor, int16_t output)
{
    if (motor != NULL)
    {
        motor->iq_output = output;
    }
}

/* 鍙戦€?LK 涓婄數/浣胯兘鍛戒护 0x88銆?*/
HAL_StatusTypeDef Motor_LkSendPowerOn(lk_motor_service_t *motor)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[8];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(tx_data, 0, sizeof(tx_data));
    tx_data[0] = MOTOR_LK_CMD_POWER_ON;
    status = Motor_SendStdFrame(motor->hcan, motor->std_id, tx_data);
    if (status == HAL_OK)
    {
        motor->output_enabled = 1U;
    }

    return status;
}

/* 鍙戦€?LK 鍋滄鍛戒护 0x81銆?*/
HAL_StatusTypeDef Motor_LkSendStop(lk_motor_service_t *motor)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[8];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(tx_data, 0, sizeof(tx_data));
    tx_data[0] = MOTOR_LK_CMD_STOP;
    status = Motor_SendStdFrame(motor->hcan, motor->std_id, tx_data);
    if (status == HAL_OK)
    {
        motor->output_enabled = 0U;
    }

    return status;
}

/* 鍙戦€?LK 鐘舵€?2 鏌ヨ鍛戒护 0x9C锛岀敤浜庡埛鏂版俯搴︺€侀€熷害銆佺數娴佸拰缂栫爜鍣ㄣ€?*/
HAL_StatusTypeDef Motor_LkSendReadState2Request(lk_motor_service_t *motor)
{
    uint8_t tx_data[8];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(tx_data, 0, sizeof(tx_data));
    tx_data[0] = MOTOR_LK_CMD_READ_STATE_2;

    return Motor_SendStdFrame(motor->hcan, motor->std_id, tx_data);
}

/* 鍙戦€?LK iq/鍔涚煩鎺у埗鍛戒护 0xA1銆?*/
HAL_StatusTypeDef Motor_LkSendIqControl(lk_motor_service_t *motor)
{
    HAL_StatusTypeDef status;
    uint8_t tx_data[8];

    if ((motor == NULL) || (motor->hcan == NULL))
    {
        return HAL_ERROR;
    }

    memset(tx_data, 0, sizeof(tx_data));
    tx_data[0] = MOTOR_LK_CMD_IQ_CONTROL;
    Motor_WriteInt16LE(tx_data, 4U, motor->iq_output);

    status = Motor_SendStdFrame(motor->hcan, motor->std_id, tx_data);
    if (status == HAL_OK)
    {
        motor->output_enabled = 1U;
    }

    return status;
}

/* 杩斿洖 LK 鍦ㄧ嚎鐘舵€併€?*/
uint8_t Motor_LkIsOnline(const lk_motor_service_t *motor)
{
    if ((motor == NULL) || (motor->online == 0U))
    {
        return 0U;
    }

    return Motor_IsRecent(motor->last_update_tick);
}

/*
 * 澶勭悊骞跺垎鍙戜竴甯?CAN 鎺ユ敹鏁版嵁銆? * BSP 鐨?CAN FIFO 鍥炶皟浼氭妸 HAL 璇诲嚭鐨?rx_header/rx_data 浼犺繘鏉ャ€? */
void Motor_ProcessCanMessage(CAN_HandleTypeDef *hcan,
                             const CAN_RxHeaderTypeDef *rx_header,
                             const uint8_t rx_data[8])
{
    uint8_t i;

    if ((hcan == NULL)
     || (rx_header == NULL)
     || (rx_data == NULL)
     || (rx_header->IDE != CAN_ID_STD)
     || (rx_header->RTR != CAN_RTR_DATA)
     || (rx_header->DLC < 7U))
    {
        return;
    }

    for (i = 0U; i < MOTOR_MAX_GM6020_COUNT; ++i)
    {
        if ((gm6020_registry[i] != NULL)
         && (gm6020_registry[i]->hcan == hcan)
         && (gm6020_registry[i]->feedback_std_id == rx_header->StdId))
        {
            Motor_UpdateDjiFeedback(&gm6020_registry[i]->feedback, rx_data);
            Motor_CopyDjiFeedbackToGm6020(gm6020_registry[i]);
            Motor_TouchOnline(&gm6020_registry[i]->online, &gm6020_registry[i]->last_update_tick);
            return;
        }
    }

    for (i = 0U; i < MOTOR_MAX_M3508_COUNT; ++i)
    {
        if ((m3508_registry[i] != NULL)
         && (m3508_registry[i]->hcan == hcan)
         && (m3508_registry[i]->feedback_std_id == rx_header->StdId))
        {
            Motor_UpdateDjiFeedback(&m3508_registry[i]->feedback, rx_data);
            Motor_CopyDjiFeedbackToM3508(m3508_registry[i]);
            Motor_TouchOnline(&m3508_registry[i]->online, &m3508_registry[i]->last_update_tick);
            return;
        }
    }

    if (rx_header->DLC < 8U)
    {
        return;
    }

    for (i = 0U; i < MOTOR_MAX_LK_COUNT; ++i)
    {
        if ((lk_registry[i] != NULL)
         && (lk_registry[i]->hcan == hcan)
         && (lk_registry[i]->std_id == rx_header->StdId))
        {
            if ((rx_data[0] == MOTOR_LK_CMD_READ_STATE_2)
             || (rx_data[0] == MOTOR_LK_CMD_IQ_CONTROL))
            {
                Motor_UpdateLkState2(lk_registry[i], rx_data);
            }
            else if (rx_data[0] == MOTOR_LK_CMD_POWER_ON)
            {
                lk_registry[i]->output_enabled = 1U;
            }
            else if (rx_data[0] == MOTOR_LK_CMD_STOP)
            {
                lk_registry[i]->output_enabled = 0U;
            }

            Motor_TouchOnline(&lk_registry[i]->online, &lk_registry[i]->last_update_tick);
            return;
        }
    }
}
