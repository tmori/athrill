#ifndef _MPU_OPS_H_
#define _MPU_OPS_H_

#include "std_types.h"

extern Std_ReturnType mpu_get_data8(CoreIdType core_id, uint32 addr, uint8 *data);
extern Std_ReturnType mpu_get_data16(CoreIdType core_id, uint32 addr, uint16 *data);
extern Std_ReturnType mpu_get_data32(CoreIdType core_id, uint32 addr, uint32 *data);

extern Std_ReturnType mpu_put_data8(CoreIdType core_id, uint32 addr, uint8 data);
extern Std_ReturnType mpu_put_data16(CoreIdType core_id, uint32 addr, uint16 data);
extern Std_ReturnType mpu_put_data32(CoreIdType core_id, uint32 addr, uint32 data);

extern Std_ReturnType mpu_get_pointer(CoreIdType core_id, uint32 addr, uint8 **data);


#endif /* _MPU_OPS_H_ */
