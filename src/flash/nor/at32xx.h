
#ifndef AT32_XX_H
#define AT32_XX_H

#define NUM_OF_MCU_TYPE                   0x0B

#define AT32F403XX_ID                     0x02
#define AT32F413XX_ID                     0x04
#define AT32F415XX_ID                     0x05
#define AT32F403AXX_ID                    0x07
#define AT32F407XX_ID                     0x08
#define AT32F421XX_ID                     0x09
#define AT32F435XX_ID                     0x0D
#define AT32F437XX_ID                     0x0E
#define AT32F425XX_ID                     0x0F
#define AT32L021XX_ID                     0x10
#define AT32WB415XX_ID                    0x11

/* at32 mcu type*/
typedef struct mcu_type_info_
{
	uint8_t        project_id;
	uint32_t       flash_bank1_reg;
	uint32_t       flash_bank2_reg;
	uint32_t       usd_addr;
	const char     name[32];
}mcu_type_info;


#if 0
static mcu_type_info mcu_type[NUM_OF_MCU_TYPE] = 
{
    //project id             project id addr   flash size addr       pid address      bank1 base    bank2 base    bank3 base     usd addr        usd size
//	{AT32F403XX_ID,   0x1FFFF7F3,    0x1FFFF7E0,    0xE0042000,  0x40022000, 0x40022040, 0x40022080, 0x1FFFF800,   48,     "AT32F403 series"},
//	{AT32F413XX_ID,   0x1FFFF7F3,    0x1FFFF7E0,    0xE0042000,  0x40022000, 0,          0x40022080, 0x1FFFF800,   48,     "AT32F413 series"},
//	{AT32F415XX_ID,   0x1FFFF7F3,    0x1FFFF7E0,    0xE0042000,  0x40022000, 0,          0,          0x1FFFF800,   1024,   "AT32F415 series"},
	{AT32F403AXX_ID,  0x1FFFF7F3,    0x1FFFF7E0,    0xE0042000,  0x40022000, 0x40022040, 0x40022080, 0x1FFFF800,   48,     "AT32F403A series"},
//	{AT32F407XX_ID,   0x1FFFF7F3,    0x1FFFF7E0,    0xE0042000,  0x40022000, 0x40022040, 0x40022080, 0x1FFFF800,   48,     "AT32F407 series"},
//	{AT32F421XX_ID,   0x1FFFF7F3,    0x1FFFF7E0,    0xE0042000,  0x40022000, 0,          0,          0x1FFFF800,   512,    "AT32F421 series"},
//	{AT32F435XX_ID,   0x1FFFF7F3,    0x1FFFF7E0,    0xE0042000,  0x40023C00, 0x40023C40, 0,          0x1FFFFC00,   512,    "AT32F435 series"},
//	{AT32F437XX_ID,   0x1FFFF7F3,    0x1FFFF7E0,    0xE0042000,  0x40023C00, 0x40023C40, 0,          0x1FFFFC00,   512,    "AT32F437 series"},
//	{AT32F425XX_ID,   0x1FFFF7F3,    0x1FFFF7E0,    0xE0042000,  0x40022000, 0,          0,          0x1FFFF800,   512,    "AT32F421 series"},
};
#endif



extern mcu_type_info at32_mcu_type[NUM_OF_MCU_TYPE];



#endif


