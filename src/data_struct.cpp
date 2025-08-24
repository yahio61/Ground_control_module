#include "data_struct.h"

uint8_t calculateCRC(data_pack_t *veriler)
{
	int check_sum = 0;
	for(int i = 1; i < sizeof(veriler->dataYapi) - 3; i++){
		check_sum += veriler->arr[i];
	}
	return (uint8_t) (check_sum % 256);
}

