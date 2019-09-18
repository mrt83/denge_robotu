#include "mpu6050.h"
#include "stm32f1xx_hal.h"
#include "timer.h"
#include "filter.h"
#include "math.h"

I2C_HandleTypeDef hi2c1;

void MPU6050_Init(uint8_t dev_addr);
void mpu6050_write(uint16_t dev_addr,uint16_t reg_addr,uint8_t *data, uint16_t size, uint32_t timeout);
void mpu6050_read(uint16_t dev_addr,uint16_t reg_addr,uint8_t *data, uint16_t size, uint32_t timeout);
void MPU6050_Task(uint8_t dev_addr);

void mpu6050_Acc_X_Axis_Read(uint8_t dev_addr);
void mpu6050_Acc_Y_Axis_Read(uint8_t dev_addr);
void mpu6050_Acc_Z_Axis_Read(uint8_t dev_addr);
void mpu6050_Gyr_X_Axis_Read(uint8_t dev_addr);
void mpu6050_Gyr_Y_Axis_Read(uint8_t dev_addr);
void mpu6050_Gyr_Z_Axis_Read(uint8_t dev_addr);
void mpu6050_Tmp_Read(uint8_t dev_addr);
void xang(void);
void yang(void);
void pitch_find(void);

int16_t xAxisValue_int=0;
int16_t yGyroValue_int=0;


float xAxisValue;
float yAxisValue;
float zAxisValue;
float xGyroValue;
float yGyroValue;
float zGyroValue;
float temperatureValue;
float temperatureValue_f;
float derece=0;
float x_ang;
float y_ang;
uint8_t gidecekler[1];

float xAxisValue_eski;
float yAxisValue_eski;
float zAxisValue_eski;

float yGyroValue_eski;
float zGyroValue_eski;

float y_ang_eski=0;
float x_ang_eski=0;

float pitch_eski=0;

float pitch;

uint8_t pitch_degissin = 0;
uint8_t once=0;
uint8_t sonra=0;


void MPU6050_Init(uint8_t dev_addr)
{


	gidecekler[0] = 0x07;

	mpu6050_write(dev_addr,SMPLRT_DIV,(uint8_t*)&gidecekler,sizeof(gidecekler),200);

	gidecekler[0] = 0x01;

	mpu6050_write(dev_addr,PWR_MGMT_1,(uint8_t*)&gidecekler,sizeof(gidecekler),200);

	gidecekler[0] = 0x00;

	mpu6050_write(dev_addr,CONFIG,(uint8_t*)&gidecekler,sizeof(gidecekler),200);

	gidecekler[0] = 0x18;

	mpu6050_write(dev_addr,GYRO_CONFIG,(uint8_t*)&gidecekler,sizeof(gidecekler),200);

	gidecekler[0] = 0x01;

	mpu6050_write(dev_addr,INT_ENABLE,(uint8_t*)&gidecekler,sizeof(gidecekler),200);

	/*mpu6050_read(dev_addr,SMPLRT_DIV,(uint8_t*)&sonra,1,200);
	mpu6050_read(dev_addr,PWR_MGMT_1,(uint8_t*)&sonra,1,200);
	mpu6050_read(dev_addr,CONFIG,(uint8_t*)&sonra,1,200);
	mpu6050_read(dev_addr,GYRO_CONFIG,(uint8_t*)&sonra,1,200);
	mpu6050_read(dev_addr,INT_ENABLE,(uint8_t*)&sonra,1,200);*/
}

void mpu6050_write(uint16_t dev_addr,uint16_t reg_addr,uint8_t *data, uint16_t size, uint32_t timeout)
{
	//HAL_I2C_Master_Transmit(&hi2c1, dev_addr, data, size, timeout);
	HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, 0x01, data, size, timeout);

}

void mpu6050_read(uint16_t dev_addr,uint16_t reg_addr,uint8_t *data, uint16_t size, uint32_t timeout)
{
	//HAL_I2C_Master_Transmit(&hi2c1, dev_addr,reg_addr, 1, timeout);

	HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, 1, data, size, timeout);

}

void MPU6050_Task(uint8_t dev_addr)
{
	if(scan2Msec)
	{


//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		mpu6050_Acc_X_Axis_Read(dev_addr);
		mpu6050_Acc_Y_Axis_Read(dev_addr);
	    mpu6050_Acc_Z_Axis_Read(dev_addr);


		//mpu6050_Gyr_X_Axis_Read(dev_addr);
		mpu6050_Gyr_Y_Axis_Read(dev_addr);
		//mpu6050_Gyr_Z_Axis_Read(dev_addr);
	//	 mpu6050_Tmp_Read(dev_addr);


		 xang();
		 yang();

		 pitch_find();


		/*mpu6050_read(dev_addr, SMPLRT_DIV, (uint8_t*) &dizi[0], 1, 200);
		mpu6050_read(dev_addr, PWR_MGMT_1, (uint8_t*) &dizi[1], 1, 200);
		mpu6050_read(dev_addr, CONFIG, (uint8_t*) &dizi[2], 1, 200);
		mpu6050_read(dev_addr, GYRO_CONFIG, (uint8_t*) &dizi[3], 1, 200);
		mpu6050_read(dev_addr, INT_ENABLE, (uint8_t*) &dizi[4], 1, 200);*/

	}
}
void mpu6050_Acc_X_Axis_Read(uint8_t dev_addr)
{
	uint8_t readDataL;
	uint8_t readDataH;
	mpu6050_read(dev_addr, 0x3B, (uint8_t*) &readDataH, 1, 200);
	mpu6050_read(dev_addr, 0x3C, (uint8_t*) &readDataL, 1, 200);
	xAxisValue = (float)((int16_t) ((readDataH) << 8) | (int16_t) (readDataL));
	if (xAxisValue > 16384)
		xAxisValue = 16384;
	else if (xAxisValue < -16384)
		xAxisValue = -16384;

	xAxisValue=xAxisValue/16384;
	//xAxisValue = (xAxisValue * (uint16_t) 90) / (uint16_t) 16384;

	xAxisValue=medianFilterAppend(xAxisValue_eski,xAxisValue, 0.3,0.6); //saniyede en fazla 30 derece değişim olabileceği kabul edildi.
	xAxisValue_eski=xAxisValue;

}
void mpu6050_Acc_Y_Axis_Read(uint8_t dev_addr) {
	uint8_t readDataL;
	uint8_t readDataH;
	mpu6050_read(dev_addr, 0x3D, (uint8_t*) &readDataH, 1, 200);
	mpu6050_read(dev_addr, 0x3E, (uint8_t*) &readDataL, 1, 200);
	yAxisValue = (float)((int16_t) ((readDataH) << 8) | (int16_t) (readDataL));
	if (yAxisValue > 16384)
		yAxisValue = 16384;
	else if (yAxisValue < -16384)
		yAxisValue = -16384;

	yAxisValue=yAxisValue/16384;

	yAxisValue=medianFilterAppend(yAxisValue_eski,yAxisValue, 0.3,0.6); //1 msaniyede en fazla 30 derece değişim olabileceği kabul edildi.
	yAxisValue_eski=yAxisValue;
	//yAxisValue = (yAxisValue * (uint16_t) 90) / (uint16_t) 16384;
}
void mpu6050_Acc_Z_Axis_Read(uint8_t dev_addr) {
	int8_t readDataL;
	int8_t readDataH;
	mpu6050_read(dev_addr, 0x3F, (uint8_t*) &readDataH, 1, 200);
	mpu6050_read(dev_addr, 0x40, (uint8_t*) &readDataL, 1, 200);
	zAxisValue = (float)((int16_t) ((readDataH) << 8) | (int16_t) (readDataL));
	if (zAxisValue > 16384)
		zAxisValue = 16384;
	else if (zAxisValue < -16384)
		zAxisValue = -16384;

	zAxisValue=zAxisValue/16384;

	if(zAxisValue<0.01)
		zAxisValue=zAxisValue_eski;

	zAxisValue_eski=zAxisValue;

	zAxisValue=medianFilterAppend(zAxisValue_eski,zAxisValue, 0.3,0.6); //en fazla 30 derece değişim olabileceği kabul edildi.
	zAxisValue_eski=zAxisValue;
	//zAxisValue = (zAxisValue * (uint16_t) 90) / (uint16_t) 16384;
}


void mpu6050_Gyr_X_Axis_Read(uint8_t dev_addr)
{
	int8_t readDataL;
	int8_t readDataH;
	mpu6050_read(dev_addr,0x43,(uint8_t*)&readDataH,1,200);
	mpu6050_read(dev_addr,0x44,(uint8_t*)&readDataL,1,200);
	xGyroValue = (int16_t)(readDataH) << 8 | (int16_t)(readDataL);
	xGyroValue = xGyroValue / (uint16_t) 131;
}
void mpu6050_Gyr_Y_Axis_Read(uint8_t dev_addr)
{
	int8_t readDataL;
	int8_t readDataH;
	mpu6050_read(dev_addr,0x45,(uint8_t*)&readDataH,1,200);
	mpu6050_read(dev_addr,0x46,(uint8_t*)&readDataL,1,200);
	yGyroValue = (float) ( (int16_t)((readDataH) << 8) | (int16_t)(readDataL));
	yGyroValue = (float)yGyroValue / (float)131;

	yGyroValue=medianFilterAppend(yGyroValue_eski,yGyroValue, 0.3 ,0.25); //saniyede en fazla 30 derece değişim olabileceği kabul edildi.
    yGyroValue_eski=yGyroValue;


}
void mpu6050_Gyr_Z_Axis_Read(uint8_t dev_addr)
{
	int8_t readDataL;
	int8_t readDataH;
	mpu6050_read(dev_addr,0x47,(uint8_t*)&readDataH,1,200);
	mpu6050_read(dev_addr,0x48,(uint8_t*)&readDataL,1,200);
	zGyroValue = (int16_t)(readDataH) << 8 | (int16_t)(readDataL);
	zGyroValue = zGyroValue / (uint16_t) 131;
}

void xang(void)

{
	x_ang=atan(yAxisValue/(sqrt(pow(xAxisValue,2)+pow(zAxisValue,2))))*57296/1000;

	//x_ang=medianFilterAppend(x_ang_eski,x_ang, 0.3,20);
	//x_ang_eski=x_ang;
}

void yang(void)
{
	y_ang=atan(xAxisValue/(sqrt(pow(yAxisValue,2)+pow(zAxisValue,2))))*57296/1000;

	//y_ang=medianFilterAppend(y_ang_eski,y_ang, 0.3,20);
	//y_ang_eski=y_ang;
}

void pitch_find(void)
{
	int8_t ara_deger=0;

	pitch=pitch+yGyroValue*0.002;   /// okuma zamanına göre burası da değişecek.*************************

	pitch=0.5*(pitch)+0.5*(y_ang);

	//pitch=medianFilterAppend(pitch_eski,pitch, 0.8,0.25);

	//pitch_eski=pitch;


   // pitch=pitch-aci_kalib;

	/*if (pitch < 5 && pitch > -5)
	{
		pitch = 0;
	}*/
   // pitch=pitch/5;

    ara_deger=round(pitch);

    pitch=(float)ara_deger;




    	/*if (pitch < aci_kalib && pitch > -aci_kalib)
		{
			pitch = 0;
		}
*/


   // pitch_eski=pitch;





    //pitch



    //pitch=y_ang-1.5;

}

void mpu6050_Tmp_Read(uint8_t dev_addr)
{
	//uint8_t sicak[2];

	uint8_t readDataL;
	uint8_t readDataH;
	mpu6050_read(dev_addr,0x41,(uint8_t*)&readDataH,1,200);
	mpu6050_read(dev_addr,0x42,(uint8_t*)&readDataL,1,200);
	temperatureValue = (float)((int16_t)((readDataH) << 8) | (int16_t)(readDataL));
	temperatureValue=(temperatureValue/(float)340) + (float)36.53;
}
