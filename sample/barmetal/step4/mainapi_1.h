/*******************************************************************************************/
/*  File Name      : mainapi_1.h                                                           */
/*  Contents       : The main header file of FlexRay node1.                                */
/*  Author         : NEC Electronics Corporation                                           */
/*  History        :                                                                       */
/*           Date         Version   Auther                       Note                      */
/*           2008.01.31   V01.00    NEC Electronics Corporation  New creation              */
/*                                                                                         */
/*******************************************************************************************/
/*******************************************************************************************/
/* プロトタイプ宣言                                                                        */
/*******************************************************************************************/
static void Set_MsgNoBuff(void);

/*******************************************************************************************/
/* define宣言                                                                              */
/*******************************************************************************************/
/* 自ノード番号 */
#define NODE_NO                         NODE_NO1

/* メッセージ送信バッファ番号一覧 */
/* (clusterNode_1.hのBufferHeader_u16[128][10]のメッセージバッファ番号) */
#define SF_SEND_MSGID_SENDCNT           ((u08)0x0)    /* 送信回数フレーム(node1)        */
#define SF_SEND_MSGID_LED               ((u08)0x3)    /* LEDデータフレーム(node1)       */
#define DF_SEND_MSGID_NMI_A             ((u08)0x6)    /* NMI押下回数フレーム chA(node1) */
#define DF_SEND_MSGID_NMI_B             ((u08)0x7)    /* NMI押下回数フレーム chB(node1) */

/* メッセージ受信バッファ番号一覧 */
/* (clusterNode_1.hのBufferHeader_u16[128][10]のメッセージバッファ番号) */
#define SF_REC1_MSGID_SENDCNT           ((u08)0x1)    /* 送信回数フレーム(node2)        */
#define SF_REC2_MSGID_SENDCNT           ((u08)0x2)    /* 送信回数フレーム(node3)        */
#define SF_REC1_MSGID_LED               ((u08)0x4)    /* LEDデータフレーム(node2)       */
#define SF_REC2_MSGID_LED               ((u08)0x5)    /* LEDデータフレーム(node3)       */
#define DF_REC1_MSGID_NMI_A             ((u08)0x8)    /* NMI押下回数フレーム chA(node2) */
#define DF_REC1_MSGID_NMI_B             ((u08)0x9)    /* NMI押下回数フレーム chB(node2) */
#define DF_REC2_MSGID_NMI_A             ((u08)0xa)    /* NMI押下回数フレーム chA(node3) */
#define DF_REC2_MSGID_NMI_B             ((u08)0xb)    /* NMI押下回数フレーム chB(node3) */
