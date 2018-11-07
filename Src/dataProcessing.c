#include "dataProcessing.h"
#include "tim.h"
#include "modbusMaster.h"

void dataProcessing() {

	switch (sendCountFlag)
	{
	case 0:
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		break;
	case 5:
		break;
	case 6:
		break;
	case 7:
		break;
	case 8:
		break;
	default:
		break;
	}
}

static void read01By04() {

	sendDataMaster04(1, 9, 8);
}

static void read01By03() {

	sendDataMaster03(1, 0, 8);
}

static void read02By03() {

	sendDataMaster03(2, 0, 7);
}

static void read03By03() {

	sendDataMaster03(2, 0, 7);
}
