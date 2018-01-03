#include "mpu.h"
#include "cpu_config.h"

#define MPU_ADDRESS_REGION_MASK_PH		0x03FFFFFF

#define MPU_ADDRESS_REGION_SIZE_INX_ROM		(1024U * 512U)
#define MPU_ADDRESS_REGION_SIZE_INX_RAM		(1024U * 10240U)
#define MPU_ADDRESS_REGION_SIZE_INX_INTC	(0xFFFFF1FB - 0xFFFFF100)
#define MPU_ADDRESS_REGION_SIZE_INX_SERIAL	(0xFFFFFA78 - 0xFFFFFA00)
#define MPU_ADDRESS_REGION_SIZE_INX_TIMER	(0xFFFFF5B0 - 0xFFFFF590)
#define MPU_ADDRESS_REGION_SIZE_INX_CPU		(1024U * 1024U)
#define MPU_ADDRESS_REGION_SIZE_INX_PH0		(1024U * 4U)
#define MPU_ADDRESS_REGION_SIZE_INX_PH1		(1024U * 12U)

static uint8 memory_data_ROM[MPU_ADDRESS_REGION_SIZE_INX_ROM];
static uint8 memory_data_RAM[MPU_ADDRESS_REGION_SIZE_INX_RAM];

static uint8 memory_data_CPU[MPU_ADDRESS_REGION_SIZE_INX_CPU * CPU_CONFIG_CORE_NUM];
static uint8 memory_data_INTC[MPU_ADDRESS_REGION_SIZE_INX_INTC * CPU_CONFIG_CORE_NUM];
static uint8 memory_data_SERIAL[MPU_ADDRESS_REGION_SIZE_INX_SERIAL];
static uint8 memory_data_TIMER[MPU_ADDRESS_REGION_SIZE_INX_TIMER];
static uint8 memory_data_PH0[MPU_ADDRESS_REGION_SIZE_INX_PH0];
static uint8 memory_data_PH1[MPU_ADDRESS_REGION_SIZE_INX_PH1];

extern MpuAddressRegionOperationType	serial_memory_operation;
extern MpuAddressRegionOperationType	timer_memory_operation;
extern MpuAddressRegionOperationType	intc_memory_operation;
extern MpuAddressRegionOperationType	cpu_register_operation;

MpuAddressMapType mpu_address_map = {
		.map = {
				/*
				 * INDEX 0:ROM
				 */
				{
						.type		= GLOBAL_MEMORY,
						.permission	= MPU_ADDRESS_REGION_PERM_ALL,
						.start		= 0x00000000,
						.size		= MPU_ADDRESS_REGION_SIZE_INX_ROM,
						.mask		= MPU_ADDRESS_REGION_MASK_ALL,
						.data		= memory_data_ROM,
						.ops		= &default_memory_operation
				},
				/*
				 * INDEX 1:RAM
				 */
				{
						.type		= GLOBAL_MEMORY,
						.permission	= MPU_ADDRESS_REGION_PERM_ALL,
						.start		= 0x05FF0000,
						.size		= MPU_ADDRESS_REGION_SIZE_INX_RAM,
						.mask		= MPU_ADDRESS_REGION_MASK_ALL,
						.data		= memory_data_RAM,
						.ops		= &default_memory_operation
				},
				/*
				 * INDEX 2:DEVICE(割込みコントローラ)
				 */
				{
						.type		= DEVICE,
						.permission	= MPU_ADDRESS_REGION_PERM_ALL,
						.start		= 0x03FFF100,
						.size		= MPU_ADDRESS_REGION_SIZE_INX_INTC,
						.mask		= MPU_ADDRESS_REGION_MASK_PH,
						.data		= memory_data_INTC,
						.ops		= &intc_memory_operation
				},
				/*
				 * TIMER
				 */
				{
						.type		= DEVICE,
						.permission	= MPU_ADDRESS_REGION_PERM_ALL,
						.start		= 0x03FFF590,
						.size		= MPU_ADDRESS_REGION_SIZE_INX_TIMER,
						.mask		= MPU_ADDRESS_REGION_MASK_PH,
						.data		= memory_data_TIMER,
						.ops		= &timer_memory_operation
				},

				/*
				 * SERIAL
				 */
				{
						.type		= DEVICE,
						.permission	= MPU_ADDRESS_REGION_PERM_ALL,
						.start		= 0x03FFFA00,
						.size		= MPU_ADDRESS_REGION_SIZE_INX_SERIAL,
						.mask		= MPU_ADDRESS_REGION_MASK_PH,
						.data		= memory_data_SERIAL,
						.ops		= &serial_memory_operation
				},


				/*
				 * INDEX :DEVICE(その他内蔵周辺I/O領域)
				 */
				{
						.type		= DEVICE,
						.permission	= MPU_ADDRESS_REGION_PERM_ALL,
						.start		= 0x03FFF000,
						.size		= MPU_ADDRESS_REGION_SIZE_INX_PH0,
						.mask		= MPU_ADDRESS_REGION_MASK_PH,
						.data		= memory_data_PH0,
						.ops		= &default_memory_operation
				},
				/*
				 * INDEX :DEVICE(プログラマブル周辺I/O領域)
				 */
				{
						.type		= DEVICE,
						.permission	= MPU_ADDRESS_REGION_PERM_ALL,
						.start		= 0x03FEC000,
						.size		= MPU_ADDRESS_REGION_SIZE_INX_PH1,
						.mask		= MPU_ADDRESS_REGION_MASK_PH,
						.data		= memory_data_PH1,
						.ops		= &default_memory_operation
				},


				/*
				 * INDEX :CPUレジスタ(デバッグ用)
				 */
				{
						.type		= DEVICE,
						.permission	= MPU_ADDRESS_REGION_PERM_ALL,
						.start		= CPU_CONFIG_DEBUG_REGISTER_ADDR,
						.size		= MPU_ADDRESS_REGION_SIZE_INX_CPU,
						.mask		= MPU_ADDRESS_REGION_MASK_ALL,
						.data		= memory_data_CPU,
						.ops		= &cpu_register_operation
				},
		}
};
