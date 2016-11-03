
#include <localdef.h>

int main()
{

	//init the FLASH memory
	vRM4_FLASH__Init();

	//int the I2C
	vRM4_I2C_USER__Init();

	//init the TSYS01
	vTSYS01__Init();


	while(1)
	{

		//process any temp sensor tasks.
		vTSYS01__Process();

	}
}



