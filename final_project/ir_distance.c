#include "ir_distance.h"

#include "adc.h"
#include "Global.h"

float get_ir_cm(char sensor)
{
	if(sensor == 'a')
	{
		float reading = read_adc(IR_A_PORT);
		float distance = A_A*pow(A_B,reading);
		return distance;
	}
	else if(sensor == 'b')
	{
		float reading = read_adc(IR_B_PORT);
		float distance = B_A*pow(B_B,reading);
		return distance;
	}

	return -69.0;
}


float get_ir_cm_base(char sensor)
{
	float distance = get_ir_cm(sensor);
	distance = IR_TO_SIDEPLATE - distance - WIDTH_BLOCK/2.0 + BASE_TO_SIDEPLATE + FUDGE_FACTOR;
	return distance;
}
