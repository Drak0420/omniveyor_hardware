#ifndef CANOPEN_COB_ID_H
#define CANOPEN_COB_ID_H

/* Definition for canopen communication object identifiers (COB-ID) */
#define COB_ID_NMT_RX			(0x000)
#define COB_ID_SYNC				(0x080)
#define COB_ID_EMCY_TX_BASE		(0x080) // Emergency 
#define COB_ID_TIME				(0x100)
#define COB_ID_TPDO_BASE		(0x080)
#define COB_ID_RPDO_BASE		(0x100)
#define COB_ID_SDO_TX_BASE		(0x580)
#define COB_ID_SDO_RX_BASE		(0x600)
#define COB_ID_NMT_EC_TX_BASE	(0x700) // Heartbeat 

/* Definition of macros to calculate COB-ID for a given Node ID */
#define COB_ID_EMCY_TX(Node_ID)					((Node_ID) + COB_ID_EMCY_TX_BASE)
#define COB_ID_TPDO(Node_ID, PDO_NO)			((Node_ID) + COB_ID_TPDO_BASE +	((PDO_NO) << 8))
#define COB_ID_RPDO(Node_ID, PDO_NO)			((Node_ID) + COB_ID_RPDO_BASE +	((PDO_NO) << 8))	
#define COB_ID_SDO_TX(Node_ID)					((Node_ID) + COB_ID_SDO_TX_BASE)
#define COB_ID_SDO_RX(Node_ID)					((Node_ID) + COB_ID_SDO_RX_BASE)
#define COB_ID_NMT_EC_TX(Node_ID)				((Node_ID) + COB_ID_NMT_EC_TX_BASE)

#define COB_ID_TPDO1_BASE       (0x180)
#define COB_ID_TPDO2_BASE       (0x280)
#define COB_ID_TPDO3_BASE       (0x380)
#define COB_ID_TPDO4_BASE       (0x480)

#endif
