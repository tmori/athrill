/*******************************************************************************************/
/*  File Name      : typedefine.h                                                          */
/*  Contents       : typedefine                                                            */
/*  Author         : NEC Electronics Corporation                                           */
/*  History        :                                                                       */
/*           Date         Version   Auther                       Note                      */
/*           2008.01.31   V01.00    NEC Electronics Corporation  New creation              */
/*                                                                                         */
/*******************************************************************************************/
/*******************************************************************************************/
/* アプリケーション/ドライバ/API共通 define宣言                                            */
/*******************************************************************************************/
typedef unsigned char   u08;
typedef signed   char   s08;
typedef unsigned short  u16;
typedef signed   short  s16;
typedef unsigned long   u32;
typedef signed   long   s32;
typedef unsigned char   BOOL;

#define ON          ((u08)1)
#define OFF         ((u08)0)
#define CLEAR       ((u08)0)

/* 関数ステータス戻り値(u08) */
typedef enum
{
    FLEXRAY_STATUS_OK                       = (u08)0x0,
    FLEXRAY_STATUS_BUSY_TIME_OUT            = (u08)0x3,
    DEFAULT_CONFIG_STATE                    = (u08)0x00,
    READY_STATE                             = (u08)0x01,
    NORMAL_ACTIVE_STATE                     = (u08)0x02,
    NORMAL_PASSIVE_STATE                    = (u08)0x03,
    HALT_STATE                              = (u08)0x04,
    MONITOR_MODE_STATE                      = (u08)0x05,
    CONFIG_STATE                            = (u08)0x0F,
    WAKEUP_STANDBY_STATE                    = (u08)0x10,
    WAKEUP_LISTEN_STATE                     = (u08)0x11,
    WAKEUP_SEND_STATE                       = (u08)0x12,
    WAKEUP_DETECT_STATE                     = (u08)0x13,
    STARTUP_PREPARE_STATE                   = (u08)0x20,
    COLDSTART_LISTEN_STATE                  = (u08)0x21,
    COLDSTART_COLLISION_RESOLUTION_STATE    = (u08)0x22,
    COLDSTART_CONSISTENCY_CHECK_STATE       = (u08)0x23,
    COLDSTART_GAP_STATE                     = (u08)0x24,
    COLDSTART_JOIN_STATE                    = (u08)0x25,
    INTEGRATION_COLDSTART_CHECK_STATE       = (u08)0x26,
    INTEGRATION_LISTEN_STATE                = (u08)0x27,
    INTEGRATION_CONSISTENCY_CHECK_STATE     = (u08)0x28,
    INITIALIZE_SCHEDULE_STATE               = (u08)0x29,
    ABORT_STARTUP_STATE                     = (u08)0x2A,
    NO_STATE                                = (u08)0xEE,
    FUNCTION_PARAMETER_ERR                  = (u08)0xFF
}FrReturnType;

/*******************************************************************************************/
/* FlexRay Data Buffer                                                                     */
/*******************************************************************************************/
/* スタティック・ダイナミックフレーム共通のペイロード長 */
#define PAYLOAD_SIZE_U32        ((u08)1)
#define PAYLOAD_SIZE_U08        PAYLOAD_SIZE_U32 * 4
#define PAYLOAD_SIZE            PAYLOAD_SIZE_U08 / 2

/* 受信バッファ構造体 */
typedef struct
{
    u32 m_Hed1_u32;                     /* Header1 data                  */
    u32 m_Hed2_u32;                     /* Header2 (Payload data Length) */
    u32 m_Hed3_u32;                     /* Header3 data                  */
    u32 m_Mbs_u32;                      /* Header4 data                  */
    u32 m_aFrame_u32[PAYLOAD_SIZE_U32]; /* Payload data                  */
}FrameBuffer;

