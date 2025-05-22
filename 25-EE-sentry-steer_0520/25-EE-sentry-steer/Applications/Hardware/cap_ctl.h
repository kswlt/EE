
#define __CAP_H__
#ifdef __CAP_H__

#include "stdint.h"

typedef struct // ��?��Y���䨬??��11��?
{
	uint8_t set_max_power; // �����
	uint8_t cache_energy;  // �������

	float remain_vol;	   // ʣ���ѹ
	float prediect_energy; // Ԥ������ 0 - 100%
} cap_t;

extern cap_t cap;

void cap_handle_message(uint8_t data[8]);
void cap_update(void);
int cap_set_power(uint8_t set);
float cap_get_remain_vol(void);
float cap_get_predict_energy(void);
#endif
