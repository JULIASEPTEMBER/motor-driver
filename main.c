#include "stm32f10x.h"
#include "./systick/wyj_systick.c.h"
#include "./usart/wyj_usart.h"
#include "./485/wyj_485.h"
#include "./can/wyj_can.h"
#include "./resource/resource.h"



#define	MOT_U_PWM PBout(10)
#define	MOT_U_ENA PAout( 7)
#define	MOT_V_PWM PAout( 6)
#define	MOT_V_ENA PAout( 5)
#define	MOT_W_PWM PAout( 4)
#define	MOT_W_ENA PAout( 3)

#define	HALL_U PAin(11)
#define	HALL_V PAin(12)
#define	HALL_W PAin(10)

#define	LED_RED    PBout(15)
#define	LED_GREEN  PAout( 8)
#define	GET_ADC1   PAin ( 1)
#define	GET_ADC2   PAin ( 2)
#define	LMP_ENABLE PAout( 9)




#define	SW_4_1 PBin ( 5)
#define	SW_4_2 PBin ( 4)
#define	SW_4_3 PBin ( 3)
#define	SW_4_4 PAin (15)


/*¹¦ÄÜº¯Êý*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
static void Car_Network_Send(u8 kind);  //ÍøÂçÍ¨ÐÅ·¢ËÍÊý¾Ý¼°ÐÄÌø
static void BreakdownInspect(u8 flag);//¹ÊÕÏ¼ì²â
static void Variate_Init(void);//È«¾Ö²ÎÊý³õÊ¼»¯
static void Car_Register(void);//»úÆ÷×¢²á
static void Run_Service(void);//ÔËÓªÏûÏ¢
static u8 ON_OFF_ABS(void);//ÈýÌ¬°´Å¥
static void Car_State(void);//Ï´³µ»ú×´Ì¬
static void Pay_Flag(void);//Ö§¸¶




static void Car_Run();//³µÌåÔËÐÐ
static u8 Car_Init_Inquirt();//³µÌå¸´Î»Ñ¯ÎÊ
static u8 Car_Init();//³µÌå¸´Î»

static u8 Button_timer(u8 kind);//°´Å¥Êä³öÊ±¼ä¼ì²â
static void Voice_Transmit(u8 kind);//ÓïÒô²¥±¨

static u8 ScanCodeInspect(void);//É¨Âë¼ì²â
static u8 Start(void);//Ï´³µ¼ì²â

/*------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*²ÎÊý±äÁ¿*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
extern Portray_State_Struct  Portray_State[1];//ÊäÈëÊä³ö¿ØÖÆÊý×é
Car_Main_Variate cmv;//È«¾Ö±äÁ¿½á¹¹Ìå


//¿ª»ú×¢²á  Ï´³µ»ú×´Ì¬  ¹ÊÕÏ  »úÆ÷ÊÇ·ñÓªÒµ  Ö§¸¶È·ÈÏ  »Ø¸´³µÅÆ  ÊÇ·ñÄÜÖ§¸¶  ÈýÌ¬°´Å¥  ÐÄÌø  ÊÇ·ñÄÜ¸´Î»
uint8_t * network_send_data[15] = {0, "FS4", "GL2", "FS1", "GL1", "FS2", "YH_?", "YH2", "GL0", "XT", "FS3"}; //2020-01-07 CHANGE OPEN TO FS4
//ÑÓÊ±Êý×é£¨000000000  ³µÌåÍ£ºÃÊ±¼ä¼ì²â  Æô¶¯  Í£Ö¹  ¼±Í££©
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
uint8_t OutputCombinedBuffer[1];//ÖØ×éµÄÊý¾Ý 
u8 flag_shutup = 0;//±Õ×ì
u8 idSave[30];//´æ´¢µ±Ç°³µÅÆÊ¶±ðidºÅÂë
u8 wantReply;//ÐèÒªÏòÎ²°Í×·¼ÓÒ»¸öip

DMA_InitTypeDef            DMA_InitStructure; 
u16 ADC_ConvertedValue[10];
u8 flagCurrentTestingState;//infomation of current test 
int nPWMH, nPWML;
/**
* @brief  Ö÷º¯Êý
* @param  ÎÞ  
* @retval ÎÞ
*/
u16 gotBuffer[1];
#define Count			100
u8 rev_buf[1];//½ÓÊÕ»º³åÇø
 
int main(void)
{
	u8 output[3];
	int totalLen;
	int count;
	int flagGoInto;
	int countGot;
	int i, j;
	int nStandardSpeed, nFullPWM;
	u8 state;
	u8 dir;
	int RunStack; 
	int nDelta;
	int nLastState;
	int StatedirListNor[6] =  {3, 1, 5, 4, 6, 2};
	int StatedirListCW[10]  ;
	int StatedirListCCW[10] ;
	//int sumAll[2];//sum up the total
	int64_t sumAll;
	u16 gotInfo, gotInfoRem;//got into buffer
	u16 Level_Jump;//valve height
	u16 TargetLevel, AllowDelta;//the real touching current of hor brush, and the space between sending and waiting

	GPIO_InitTypeDef GPIO_InitStructure;

	Project_GPIOInit();//GPIO³õÊ¼»¯
	//USARTx_Config(1);  //USART³õÊ¼»¯

	//RS485_Config();    //485³õÊ¼»¯
	//RS485_3_TX_EN();  //ÒôÆµ·¢ËÍÊ¹ÄÜ
	InitTimeCounter(); //³õÊ¼»¯¼ÆÊ±Æ÷
	Delay(3000000); 
	//	Variate_Init();//È«¾Ö²ÎÊý³õÊ¼»¯ 
	//	totalLen = 5000;//count add all length of buffer


	//Portray_State[Dout__Light3].state = Portray_State[Dout__Light3].enaType;

	//Init_DMA_Ena(0);
	//Init_ADC(0);
	//USARTx_Config(3);  //USART³õÊ¼»¯

	Level_Jump = 0x200;
	flagGoInto = 0;
	gotInfoRem = 0;
	gotInfo = 0;
	countGot = 0;
	Timer_Pulse(1, 1);
	flagCurrentTestingState = 0;
	//PBout(0) = 0;
	//PBout(1) = 0;
	//while(1)
	//{
	//PBout(0) = 0;
	//PBout(1) = 0;
	//	//		Usart_SendByte(USART3, 0xBB);
	//}

	//GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4| GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	//GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;//ÍÆÍìÊä³ö
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;//ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_4 | GPIO_Pin_3 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;//ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	
	GPIO_InitStructure.GPIO_Pin   =  GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;//ÉÏÀ­ÊäÈë
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	
	GPIO_InitStructure.GPIO_Pin   =  GPIO_Pin_5 | GPIO_Pin_4 ;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;//ÉÏÀ­ÊäÈë
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

//#define	MOT_U_PWM PBout(10)
//#define	MOT_U_ENA PAout( 7)
//#define	MOT_V_PWM PAout( 6)
//#define	MOT_V_ENA PAout( 5)
//#define	MOT_W_PWM PAout( 4)
//#define	MOT_W_ENA PAout( 3)
//
//#define	HALL_U PAin(11)
//#define	HALL_V PAin(12)
//#define	HALL_W PAin(10)
//
//#define	LED_RED    PBout(15)
//#define	LED_GREEN  PAout( 8)
//#define	GET_ADC1   PAin ( 1)
//#define	GET_ADC2   PAin ( 2)
//#define	LMP_ENABLE PAout( 9)

LMP_ENABLE = 1;


	//	while(1)
	//	{
	//		
	//	MOT_U_ENA = 1;
	//	MOT_U_ENA = 0;
	//	}
	MOT_U_PWM = 0;
	MOT_U_ENA = 0;
	MOT_V_PWM = 0;
	MOT_V_ENA = 0;
	MOT_W_PWM = 0;
	MOT_W_ENA = 0;

	CAN_Config();

	//#define	MOT_U_PWM PBout(5)
	//#define	MOT_U_ENA PBout(1)
	//#define	MOT_V_PWM PBout(0)
	//#define	MOT_V_ENA PAout(7)
	//#define	MOT_W_PWM PAout(6)
	//#define	MOT_W_ENA PAout(5)
	//
	//#define	HALL_U PAin(2)
	//#define	HALL_V PAin(1)
	//#define	HALL_W PAin(0)
	dir = 1;//ORIGINAL CLOCKWISE 
	nPWMH = 50;
	nStandardSpeed = 30;
	nFullPWM = 230;
	nPWML = nFullPWM - nPWMH;
	//while(1); 
	nLastState = 0;
	
	state = 0;
	
	//int StatedirListNor[6] =  {3, 1, 5, 4, 6, 2};
	//int StatedirListCW[6]  ;
	//int StatedirListCCW[6] ;
	StatedirListCW[3] = 1;
	StatedirListCW[1] = 5;
	StatedirListCW[5] = 4;
	StatedirListCW[4] = 6;
	StatedirListCW[6] = 2;
	StatedirListCW[2] = 3;
	
	StatedirListCCW[3] = 2;
	StatedirListCCW[1] = 3;
	StatedirListCCW[5] = 1;
	StatedirListCCW[4] = 5;
	StatedirListCCW[6] = 4;
	StatedirListCCW[2] = 6;
	RunStack = 0;
	nDelta = 0;
	ResetGlobalBuffer = 0;
	
	
	LED_RED = 1;
	LED_GREEN = 1;
	
	if(SW_4_1 == 0)
	Switch_4pin = 1;
	if(SW_4_2 == 0)
	Switch_4pin = 2;
	if(SW_4_4 == 0)
	Switch_4pin = 4;
	while(0)
	{
	
		
		state = 0;
		state += HALL_U ? 1 : 0;
		state += HALL_V ? 2 : 0;
		state += HALL_W ? 4 : 0;
		state = state;
		
		if(HALL_U == 0)
			state = state;
		if(HALL_V == 0)
			state = state;
		if(HALL_W == 0)
			state = state;
		
				MOT_U_ENA = 1;
				MOT_U_PWM = 1;
				Delay(20000); 
				MOT_U_ENA = 1;
				MOT_U_PWM = 0;
				Delay(20000); 
				MOT_U_ENA = 0;
				MOT_U_PWM = 0;
				Delay(20000); 
				MOT_U_ENA = 0;
				MOT_U_PWM = 1;
				Delay(20000); 
		
				MOT_W_ENA = 1;
				MOT_W_PWM = 1;
				Delay(20000); 
				MOT_W_ENA = 1;
				MOT_W_PWM = 0;
				Delay(20000); 
				MOT_W_ENA = 0;
				MOT_W_PWM = 0;
				Delay(20000); 
				MOT_W_ENA = 0;
				MOT_W_PWM = 1;
				Delay(20000); 
				
				MOT_V_ENA = 1;
				MOT_V_PWM = 1;
				Delay(20000); 
				MOT_V_ENA = 1;
				MOT_V_PWM = 0;
				Delay(20000); 
				MOT_V_ENA = 0;
				MOT_V_PWM = 0;
				Delay(20000); 
				MOT_V_ENA = 0;
				MOT_V_PWM = 1;
				Delay(20000); 
	}
	while(1)
	{//continue;
		
		
	if(SW_4_1 == 0 && SW_4_2 == 1 && SW_4_3 == 0 && SW_4_4 == 1)
	Switch_4pin = 1;
	else if(SW_4_1 == 1 && SW_4_2 == 0 && SW_4_4 == 1)
	Switch_4pin = 2;
	else if(SW_4_1 == 0 && SW_4_2 == 0 && SW_4_4 == 1)
	Switch_4pin = 3;
	else if(SW_4_1 == 1 && SW_4_2 == 1 && SW_4_4 == 0)
	Switch_4pin = 4;
	else if(SW_4_1 == 0 && SW_4_2 == 1 && SW_4_4 == 0)
	Switch_4pin = 5;
	else 
	Switch_4pin = 0;
		
		
		
		if(ResetGlobalBuffer) {RunStack  = 0;ResetGlobalBuffer = 0;}
		if( nLastState != state)
		{
			if(nLastState)
			{
				if(StatedirListCW[nLastState] == state)//cw rotate
					RunStack ++;
				else if(StatedirListCCW[nLastState] == state)//ccw rotate
					RunStack --;
			}
			nLastState = state;
		}
		if(RunStack != CountPlace)
		{
			if(RunStack > CountPlace)
			{
			 	dir = 0;
				nDelta = RunStack - CountPlace;
				nPWMH = nStandardSpeed + 5 * (nDelta) ;
			}
			else
			{
				dir = 1;
				nDelta = CountPlace - RunStack;
				nPWMH = nStandardSpeed + 5 * (nDelta); 
			}
			nPWMH = nPWMH > nFullPWM - 20 ? nFullPWM : nPWMH;
			nPWML = nFullPWM - nPWMH;
			
			
		}
		state = 0;
		state += HALL_U ? 1 : 0;
		state += HALL_V ? 2 : 0;
		state += HALL_W ? 4 : 0;
		//if(0)
		
		if(nDelta < 6  || nDelta > 1000)
		{
				MOT_U_ENA = 0; MOT_V_ENA = 0; MOT_W_ENA = 0;
				MOT_U_PWM = 0; MOT_V_PWM = 0; MOT_W_PWM = 0;
				if(nDelta > 1000)
				{
					LED_RED = 0;
					LED_GREEN = 1;
				}
				else
				{
					LED_RED = 1;
					LED_GREEN = 1;
				}
				
				continue;
		}

					//LED_RED = 0;
	//	LED_GREEN = 0;
//continue;
		if(dir)//cw
		{
			
			switch(state)
			{
			case 3://110
				MOT_U_ENA = 0;MOT_V_ENA = 1;MOT_W_ENA = 1;
				MOT_W_PWM = 1;
				Delay(nPWMH); 
				MOT_W_PWM = 0;
				Delay(nPWML);
				break;
			case 1://100
				MOT_U_ENA = 0;MOT_V_ENA = 1;MOT_W_ENA = 1;
				MOT_V_PWM = 1;
				Delay(nPWMH); 
				MOT_V_PWM = 0;
				Delay(nPWML);
				break;
			case 5://101
				MOT_U_ENA = 1;MOT_V_ENA = 1;MOT_W_ENA = 0;
				MOT_V_PWM = 1;
				Delay(nPWMH); 
				MOT_V_PWM = 0;
				Delay(nPWML);
				break;
			case 4://001
				MOT_U_ENA = 1;MOT_V_ENA = 1;MOT_W_ENA = 0;
				MOT_U_PWM = 1;
				Delay(nPWMH); 
				MOT_U_PWM = 0;
				Delay(nPWML);
				break;
			case 6://011
				MOT_U_ENA = 1;MOT_V_ENA = 0;MOT_W_ENA = 1;
				MOT_U_PWM = 1;
				Delay(nPWMH); 
				MOT_U_PWM = 0;
				Delay(nPWML);
				break;
			case 2://010
				MOT_U_ENA = 1;MOT_V_ENA = 0;MOT_W_ENA = 1;
				MOT_W_PWM = 1;
				Delay(nPWMH); 
				MOT_W_PWM = 0;
				Delay(nPWML);
				break;
			}
		}
		else
		{
			
			switch(state)
			{
			case 3://110
				MOT_U_ENA = 1;MOT_V_ENA = 1;MOT_W_ENA = 0;
				MOT_U_PWM = 1;
				Delay(nPWMH); 
				MOT_U_PWM = 0;
				Delay(nPWML);
				break;
			case 1://100
				MOT_U_ENA = 1;MOT_V_ENA = 0;MOT_W_ENA = 1;
				MOT_U_PWM = 1;
				Delay(nPWMH); 
				MOT_U_PWM = 0;
				Delay(nPWML);
				break;
			case 5://101
				MOT_U_ENA = 1;MOT_V_ENA = 0;MOT_W_ENA = 1;
				MOT_W_PWM = 1;
				Delay(nPWMH); 
				MOT_W_PWM = 0;
				Delay(nPWML);
				break;
			case 4://001
				MOT_U_ENA = 0;MOT_V_ENA = 1;MOT_W_ENA = 1;
				MOT_W_PWM = 1;
				Delay(nPWMH); 
				MOT_W_PWM = 0;
				Delay(nPWML);
				break;
			case 6://011
				MOT_U_ENA = 0;MOT_V_ENA = 1;MOT_W_ENA = 1;
				MOT_V_PWM = 1;
				Delay(nPWMH); 
				MOT_V_PWM = 0;
				Delay(nPWML);
				break;
			case 2://010
				MOT_U_ENA = 1;MOT_V_ENA = 1;MOT_W_ENA = 0;
				MOT_V_PWM = 1;
				Delay(nPWMH); 
				MOT_V_PWM = 0;
				Delay(nPWML);
				break;
			}
		}
	}
	//while(1)
	{
		CanTxMsg TxMessage;
		for(i = 0 ;i < 8; i ++)
			TxMessage.Data[i] = i;
		//	CAN_SetMsg(&TxMessage, 8, 0x00);
		Delay(1000000);
	}
	for(i = 0; i < Count; i ++)
	{
		MOT_U_ENA = 1;

		Delay(60); 

		MOT_U_ENA = 1;

		Delay(170);
	}
	while(1)
	{
		/////////////////////////U///////////////////////////
		for(i = 0; i < Count; i ++)
		{
			MOT_U_PWM = 1;
			Delay(60); 
			MOT_U_PWM = 0;
			Delay(170);
		}
		MOT_W_ENA =  0;
		MOT_V_ENA =  1;
		for(i = 0; i < Count; i ++)
		{
			MOT_U_PWM = 1;

			Delay(60); 

			MOT_U_PWM = 0;

			Delay(170);
		}

		////////////////////////////////////////////////////
		/////////////////////////V///////////////////////////
		for(i = 0; i < Count; i ++)
		{
			MOT_V_PWM = 1;

			Delay(60); 

			MOT_V_PWM = 0;

			Delay(170);
		}
		MOT_U_ENA =  0;
		MOT_W_ENA =  1;
		for(i = 0; i < Count; i ++)
		{
			MOT_V_PWM = 1;

			Delay(60); 

			MOT_V_PWM = 0;

			Delay(170);
		}

		////////////////////////////////////////////////////
		/////////////////////////W//////////////////////////
		for(i = 0; i < Count; i ++)
		{
			MOT_W_PWM = 1;

			Delay(60); 

			MOT_W_PWM = 0;

			Delay(170);
		}
		MOT_V_ENA =  0;
		MOT_U_ENA =  1;
		for(i = 0; i < Count; i ++)
		{
			MOT_W_PWM = 1;//0x01 0x02

			Delay(60); 

			MOT_W_PWM = 0;

			Delay(170);
		}

		////////////////////////////////////////////////////
	}
	while(1)
	{
		switch(flagCurrentTestingState)
		{
		case 0://wait for standard scanning

			break;
		case 1://begin to scan the standard(got)
			Level_Jump = 0xffff & GetStandardProc();
			flagCurrentTestingState = 0;
			break;
		case 2://testing
			TestingInfoAnalyze(Level_Jump, TargetLevel, AllowDelta);
			break;
		case 3://get target level and allowed delta space
			TargetLevel = rev_buf[20] + rev_buf[21] * 256;
			AllowDelta = rev_buf[22] + rev_buf[23] * 256;
			break;
		}
	}
	while(1)
	{
		switch(flagGoInto)
		{
		case 0:
			sumAll = 0; 
			flagGoInto ++;
			countGot = 0;
			break;
		case 1: 
			sumAll += ADC_ConvertedValue[0];
			countGot ++; 

			if(Timer_Pulse(0, 1) > 5 )
				flagGoInto ++;

			break;
		case 2:
			gotInfo = sumAll / countGot;
			Usart_SendByte(USART1, 0x55); 
			Usart_SendByte(USART1, ((gotInfo) >> 0) & 0xff);
			Usart_SendByte(USART1, ((gotInfo) >> 8) & 0xff);
			Usart_SendByte(USART1, 0x0D); 
			Timer_Pulse(1, 1);
			flagGoInto = 0;
			break;
		default:
			flagGoInto = 0;
			break;
		}
	}
	while(1)
	{

		//	gotInfo = ADC_ConvertedValue[0];
		if(Timer_Pulse(0, 1) > 5 && flagGoInto == 0)
		{

			Usart_SendByte(USART1, 0x55); 
			Usart_SendByte(USART1, sumAll & 0xff);
			Usart_SendByte(USART1, (sumAll >> 8) & 0xff);
			Usart_SendByte(USART1, 0x0D); 
			Timer_Pulse(1, 1);
		}
		switch(flagGoInto)
		{
		case 0:
			flagGoInto = 1;
			sumAll = 0;
			break;
		case 1:
			for(i = 0; i < totalLen; i ++)
			{
				sumAll = sumAll + ADC_ConvertedValue[0];
			}
			/////////////////////////////
			//for(i = 0; i < totalLen; i ++)
			//{ 
			//	
			//	Usart_SendByte(USART1, 0x55); 
			//	Usart_SendByte(USART1, gotBuffer[i] & 0xff);
			//	Usart_SendByte(USART1, (gotBuffer[i] >> 8) & 0xff);
			//	Usart_SendByte(USART1, 0x0D); 
			//}
			/////////////////////////////
			flagGoInto = 2;
			break;
		case 2:

			sumAll = sumAll / totalLen;
			flagGoInto = 3;
			break;
		default:
			flagGoInto = 0;
			break;
		}
	}
	while(1)
	{ 
		//	count ++; 
		//		 if(count > 100000)
		//got = ADC_ConvertedValue[0];
		gotInfo = ADC_ConvertedValue[0];
		switch(flagGoInto)
		{
		case 0:
			if(gotInfo > Level_Jump)
			{
				flagGoInto = 1;
			}
			break;
		case 1:
			{
				for(countGot = 0; countGot < totalLen; countGot ++)
				{
					gotBuffer[countGot] = ADC_ConvertedValue[0];
					if(gotBuffer[countGot] < Level_Jump)
						break;
				}
				for(i = 0; i < countGot; i ++)
				{
					Usart_SendByte(USART1, 0x01);
					Usart_SendByte(USART1, 0x03);
					Usart_SendByte(USART1, 0x04);
					Usart_SendByte(USART1, gotBuffer[i] & 0xff);
					Usart_SendByte(USART1, (gotBuffer[i] >> 8) & 0xff);
					Usart_SendByte(USART1, 0);
					Usart_SendByte(USART1, 0);
					Usart_SendByte(USART1, 0x0D); 
				}
				flagGoInto = 0;
				break;
		default:
			flagGoInto = 0;
			break;
			}
		}
	}

	while(1)
	{
		BreakdownInspect(1);  //¹ÊÕÏÍ¨ÐÅ
		Car_Network_Send(0);  //ÍøÂçÍ¨ÐÅ·¢ËÍÊý¾Ý¼°ÐÄÌø
		Control_InputOutput();//¿ØÖÆ¹ÜÀíÊäÈëÊä³ö
		Run_Service();  //ÔËÓªÏûÏ¢
		ON_OFF_ABS();  //ÈýÌ¬°´Å¥
		Car_State();  //Ï´³µ»ú×´Ì¬
		Pay_Flag();  //Ö§¸¶ÏûÏ¢

		if(wantReply)
		{
			wantReply = 0;
			//	Delay(1000000);
			RS485_SendString(idSave + 1);
		}

		if(cmv.run_service[0]) {  //ÔËÓª
			BreakdownInspect(0);//¹ÊÕÏ¼ì²â
			Car_Run();//³µÌåÔËÐÐ
		}
	}
}


/**
* @brief  :Ö§¸¶
* @param  :ÎÞ
* @retval :ÎÞ
*/
static void Pay_Flag(void)
{
	uint8_t st1[7], st2[7],  st3[2],  st4[2]; 
	u8 bufferTemp[30];
	int i ;
	i = 0;
	st1[i] = 'Y';i ++;
	st1[i] = 'H';i ++;
	st1[i] = '1';i ++;
	st1[i] = '_';i ++;
	st1[i] = 0  ;i ++;
	i = 0;
	st2[i] = '_';i ++;
	st2[i] =   0;i ++; 
	i = 0;
	st3[i] = '1';i ++;
	st3[i] =   0;i ++; 
	i = 0;
	st4[i] = '0';i ++;
	st4[i] =   0;i ++; 

	if(cmv.pay_flag) {  //»ñÈ¡ÊÇ·ñÄÜÖ§¸¶Ñ¯ÎÊÏûÏ¢
		cmv.pay_flag = 0;


		if(cmv.start_state || cmv.run_service[0] == 0)
			network_send_data[7] = 0;
		else
			*(int*)(&network_send_data[7]) = 1;


		//cmv.pay_flag_ID[cmv.pay_flag_ID[0]+2] = data;
		//		cmv.pay_flag_ID[0]++;
		copyBuf(bufferTemp, st1);
		SendString_Jojnt(bufferTemp, &(cmv.pay_flag_ID[2]));
		copyBuf(bufferTemp, OutputCombinedBuffer);
		SendString_Jojnt(bufferTemp, st2);
		copyBuf(bufferTemp, OutputCombinedBuffer);
		SendString_Jojnt(bufferTemp, network_send_data[7] ? st3 : st4);
		network_send_data[7]  = OutputCombinedBuffer;
		//OutputCombinedBuffer 
		Voice_Transmit(1);  //²¥·ÅÓïÒô
		Car_Network_Send(7);  //·¢ËÍ×¢²áÐÅÏ¢
	}

	if(cmv.car_payment_flag[0]) {  //»ñÈ¡Ö§¸¶ÏûÏ¢
		cmv.car_payment_flag[0] = 0;

		if(cmv.payment_message[0] == '1')
			cmv.car_payment_flag[2] = 1;

		if(cmv.payment_message[3] == '1')
			cmv.wind_flag[0] = 1;
		else if(cmv.payment_message[3] == '0')
			cmv.wind_flag[0] = 0;
	}
}


/**
* @brief  :Ï´³µ»ú×´Ì¬
* @param  :ÎÞ
* @retval :ÎÞ
*/
static void Car_State(void)
{
	u8 uTempBuf[20], uTrueBuf[3], uFalseBuf[3];
	if(cmv.car_state) {  //½ÓÊÕµ½»ñÈ¡×´Ì¬ÐÅÏ¢
		cmv.car_state = 0;
		uTrueBuf[0] = '_';
		uFalseBuf[0] = '_';
		uTrueBuf[1] = '1';
		uFalseBuf[1] = '0';
		uTrueBuf[2] = 0;
		uFalseBuf[2] = 0;
		uTempBuf[0] = 0;
		cmv.car_state = 0;

		//uint8_t * SendString_Jojnt(uint8_t *str1, uint8_t *str2);
		//void copyBuf(uint8_t *Target, uint8_t *origin);
		//	network_send_data[2] = "GL2";
		copyBuf(uTempBuf, (u8*)"GL2");
		//¼ì²âÀ¯Ë®
		SendString_Jojnt(uTempBuf, cmv.trouble_flag[0] ? uFalseBuf : uTrueBuf);
		copyBuf(uTempBuf, OutputCombinedBuffer);



		//¼ì²âÅÝÄ­
		SendString_Jojnt(uTempBuf, cmv.trouble_flag[1] ? uFalseBuf : uTrueBuf);
		copyBuf(uTempBuf, OutputCombinedBuffer);

		//¼ì²â×ÔÀ´Ë®

		SendString_Jojnt(uTempBuf, cmv.trouble_flag[1] ? uTrueBuf : uTrueBuf);
		copyBuf(uTempBuf, OutputCombinedBuffer);

		//¼ì²âÏ´³µÊÇ·ñÍê³É
		SendString_Jojnt(uTempBuf, cmv.start_state ? uFalseBuf : uTrueBuf);
		copyBuf(uTempBuf, OutputCombinedBuffer);




		network_send_data[2] = OutputCombinedBuffer;
		Car_Network_Send(2);  //Í¨Öª×´Ì¬
	}
}


/**
* @brief  :ÈýÌ¬°´Å¥
* @param  :ÎÞ
* @retval :ÎÞ
*/
static u8 ON_OFF_ABS(void)
{
	if(cmv.ONOFF[0]) {  //½ÓÊÕµ½¾ø¶Ô¿ØÖÆÏûÏ¢
		cmv.ONOFF[0] = 0;
		Car_Network_Send(8);  //Í¨ÖªÒÑÊÕµ½
		cmv.wind_flag[1] = 3;
		if(cmv.ONOFF[1] == 0)
			while(!Button_timer(Dout__Stop));
		else if(cmv.ONOFF[1] == 1)
			while(!Button_timer(Dout__Start));
		else if(cmv.ONOFF[1] == 2)
			while(!Button_timer(Dout__Scram));
		return 1;
	}
	return 0;
}


/**
* @brief  :ÔËÓªÏûÏ¢
* @param  :ÎÞ
* @retval :ÎÞ
*/
static void Run_Service(void)
{
	//ÔËÓªÏûÏ¢´¦Àí
	if(cmv.run_service[1])
	{
		if(cmv.run_service[1] == 1) {  //ÔËÓªÏûÏ¢
			if(cmv.run_service[0] == 0) {
				cmv.run_service[0] = 1;  //Èç¹û´¦ÓÚ¹Ø±Õ×´Ì¬¾Í´ò¿ª
				//Variate_Init();//È«¾Ö²ÎÊý³õÊ¼»¯
			}
		}
		else if(cmv.run_service[1] == 2) { //²»ÄÜÔËÓª
			if(cmv.run_service[0] == 1)  cmv.run_service[0] = 0;  //Èç¹û´¦ÓÚ·ÇÔËÐÐ×´Ì¬¾Í¹Ø±Õ
		}

		cmv.run_service[1] = 0;  //Çå¿ÕÏûÏ¢
		Car_Network_Send(4);  //Í¨ÖªÒÑÊÕµ½
	}
}


/**
* @brief  :»úÆ÷×¢²á
* @param  :ÎÞ
* @retval :ÎÞ
*/
static void Car_Register(void)
{
	while(1)  //»úÆ÷×¢²á
	{
		cmv.car_register[1] = 1;  //Ê¹ÄÜ×¢²á½ÓÊÕ
		Car_Network_Send(1);  //·¢ËÍ×¢²áÐÅÏ¢

		if(cmv.car_register[0]) {  //×¢²áÍê³É
			cmv.car_register[1] = 0;
			cmv.car_register[0] = 0;
			Voice_Transmit(1);  //²¥·ÅÓïÒô
			break;
		}
	}
}


/**
* @brief  :È«¾Ö²ÎÊý³õÊ¼»¯
* @param  :ÎÞ
* @retval :ÎÞ
*/
static void Variate_Init(void)
{
	int i;

	cmv.car_register[1] = 0;  //½ûÖ¹×¢²á½ÓÊÕ
	cmv.car_register[0] = 0;  //Î´×¢²á

	cmv.run_service[0] = 0;  //¿ª»ú²»ÔËÓª
	cmv.run_service[1] = 0;  //ÏûÏ¢Çå¿Õ

	for(i=0; i<5; i++)
		cmv.trouble_flag[i] = 0;  //³õÊ¼»¯Ã»ÓÐ¹ÊÕÏ
	cmv.trouble[0] = 0;  //½ûÖ¹½ÓÊÕ
	cmv.trouble[1] = 0;  //½ÓÊÕµÄÊý¾Ý

	cmv.start_state = 0;  //³õÊ¼»¯Î´´¦ÓÚÏ´³µ×´Ì¬

	cmv.ONOFF[0] = 0;  //³õÊ¼»¯Î´½ÓÊÕÈýÌ¬¿ØÖÆ

	cmv.car_state = 0;  //³õÊ¼»¯Î´½ÓÊÕµ½»ñÈ¡×´Ì¬ÐÅÏ¢

	cmv.car_run_flag = 0;  //³µÌåÔËÐÐ²½Öè

	cmv.car_rest_flag[1] = 0;  //½ûÄÜ¸´Î»½ÓÊÕ
	cmv.car_rest_flag[0] = 0;  //³õÊ¼»¯²»ÄÜ¸´Î»

	cmv.pay_flag = 0;  //ÊÇ·ñÄÜÖ§¸¶»ñÈ¡ÏûÏ¢³õÊ¼»¯ÎªÃ»ÓÐ
	for(i=0; i<50; i++)
		cmv.pay_flag_ID[i] = 0;

	cmv.car_payment_flag[0] = 0;  //Î´»ñÈ¡ÏûÏ¢
	cmv.car_payment_flag[1] = 0;  //½ûÖ¹»ñÈ¡
	cmv.car_payment_flag[2] = 0;  //Î´Ö§¸¶
	for(i=0; i<5; i++)
		cmv.payment_message[i] = 0;

	cmv.wind_flag[0] = 1;  //Ä¬ÈÏÄÜ´µ·ç
	cmv.wind_flag[1] = 0;  //»ØÎ»´ÎÊý


	Timer_Pulse(1, 0);  //ÖØÖÃÊ±¼ä
}


/**
* @brief  :ÍøÂçÍ¨ÐÅ·¢ËÍÊý¾Ý¼°ÐÄÌø
* @param  :ÎÞ
* @retval :ÎÞ
*/
static void Car_Network_Send(u8 kind)
{
	int i=0, j=0;
	static u8 heartbeat_timer_count = 0;  //ÐÄÌøÊ±¼ä¼ÆÊý
	static u8 send_array[15] = {0};  //´ý·¢ËÍÁÐ±í
	static u8 send_array_count = 0;  //´ý·¢ËÍÇøÊý¾Ýµ±Ç°Î»ÖÃ

	if(kind) {  //kindÎª0±íÊ¾µ÷ÓÃ
		for(i=0; i<send_array_count; i++) {  //±éÀú´ý·¢ËÍÇø±ÜÃâÖØ¸´,×ö¼ÇÂ¼
			if(send_array[i] == kind) {
				j = 1;
				break;
			}
		}

		if(!j) {  //µ±Ç°·¢ËÍÊý¾ÝÇøÃ»ÓÐ´ËÏîÊý¾Ý(kind),Ìí¼Ó½øÈ¥
			send_array[send_array_count] = kind;
			send_array_count++;
		}
	}
	if(Timer_Pulse(0, 0)  > 15)
		flag_shutup = 0;//³¬Ê±¡¢¿ÉÒÔ½²»°

	//if(Timer_Pulse(0, 0)  == 15 && idSave[0])
	//{
	//	
	//		RS485_SendString("YH0");
	//		RS485_SendString(idSave + 1);
	//		idSave[0] = 0;
	//}


	if(Timer_Pulse(0, 0) >= 3 && flag_shutup == 0) {
		if(send_array_count > 0) {  //Êý¾ÝÇøÓÐÊý¾Ý
			RS485_TX_EN();
			//else
			RS485_SendString(network_send_data[send_array[0]]);
			//if(OutputCombinedBuffer[0])
			{ 
				//OutputCombinedBuffer[0] = 0;
			}
			RS485_RX_EN();

			for(i=0; i<send_array_count; i++) {  //Êý¾ÝÇøÍùÇ°µÝÍÆ
				send_array[i] = send_array[i+1];
			}
			send_array_count--;
		}

		heartbeat_timer_count++;
		if(heartbeat_timer_count == 20) {  //30s·¢ËÍÐÄÌø
			heartbeat_timer_count = 0;
			//Car_Network_Send(9);
		}

		Timer_Pulse(1, 0);  //ÖØÖÃÊ±¼ä
	}
}


/**
* @brief  £º³µÌåÔËÐÐ
* @param  £ºÎÞ
* @retval : ÎÞ
*/
static void Car_Run()
{
	static u8 flag = 1;

	switch(cmv.car_run_flag)
	{
	case 0: {  //µÈ´ý¸´Î»
		if(flag == 1  &&  Portray_State[Din__Car_Init].state == Portray_State[Din__Car_Init].enaType) {
			cmv.car_run_flag = 1;
		}
		else {
			if(flag == 1  &&  Car_Init_Inquirt()) {  //Ñ¯ÎÊÊÇ·ñ¸´Î»
				flag = 0;
				Voice_Transmit(1);  //²¥·ÅÓïÒô

			}
			else if(flag == 0) {  //¸´Î»
				if(Car_Init()) flag = 1;
			}
		}
			}break;

	case 1://¼ì²âÉ¨Âë
		{
			if(ScanCodeInspect())
				cmv.car_run_flag = 2;
		}break;

	case 2://³µÌåÔËÐÐ
		{
			if(Start())
				cmv.car_run_flag = 3;
		}break;

	case 3://Ï´³µ½áÊø
		{
			//	Voice_Transmit(3);  //²¥·ÅÓïÒô
			cmv.run_service[0] = 1;  //´¦ÓÚÔËÓªÇÒÄÜ½ûÖ¹×´Ì¬
			cmv.start_state = 0;  //ÊÇ·ñÏ´³µ×´Ì¬
			cmv.car_run_flag = 0;  //ÔËÐÐ²½Öè
			cmv.wind_flag[1] = 0;  //Ï´³µ·µ»Ø´ÎÊý
			Voice_Transmit(8);  //²¥·ÅÓïÒô

			Portray_State[Dout__Light1].state = 1 - Portray_State[Dout__Light1].enaType;
			Portray_State[Dout__Light2].state = 1 - Portray_State[Dout__Light2].enaType;
			Portray_State[Dout__Light3].state = Portray_State[Dout__Light3].enaType;
			while(Portray_State[Din__Car_Exist].state != 1 - Portray_State[Din__Car_Exist].enaType)
			{
				Control_InputOutput();//¿ØÖÆ¹ÜÀíÊäÈëÊä³ö

				Car_State();  //Ï´³µ»ú×´Ì¬
				if(ON_OFF_ABS())break;  //ÈýÌ¬°´Å¥
			}

			//Delay(1000000);

		}break;
	}
}


/**
* @brief  £º³µÌå¸´Î»Ñ¯ÎÊ
* @param  £ºÎÞ
* @retval : ÎÞ
*/
static u8 Car_Init_Inquirt()
{
	cmv.car_rest_flag[1] = 1;  //Ê¹ÄÜ½ÓÊÕ
	Car_Network_Send(10);  //Ñ¯ÎÊÊÇ·ñ¸´Î»

	if(cmv.car_rest_flag[0]) {  //¿ÉÒÔ¸´Î»
		cmv.car_rest_flag[1] = 0;  //½ûÄÜ½ÓÊÕ
		cmv.car_rest_flag[0] = 0;
		return 1;
	}

	return 0;
}


/**
* @brief  £º³µÌå¸´Î»
* @param  £ºÎÞ
* @retval : ÎÞ
*/
static u8 Car_Init()
{
	static u8 flag = 1;

	if(flag)  {  //¸´Î»
		while(!Button_timer(Dout__Start));
		flag = 0;
	}

	//¸´Î»Íê±Ï
	if(Portray_State[Din__Car_Init].state == Portray_State[Din__Car_Init].enaType) {
		flag = 1;
		return 1;
	}

	return 0;
}


/**
* @brief  £º¹ÊÕÏ¼ì²â
* @param  £ºÎÞ
* @retval : ÎÞ
*/
static void BreakdownInspect(u8 flag)//¹ÊÕÏ(À¯Ë®£¬ÅÝÄ­£¬·À×²¸Ë£¬ÏÞ¸ß£¬µ¹·ü)
{
	int i;
	int state = 0;
	static u8 trouble_copy[5] = {0};
	//u8 *str = "FS1";
	u8 strCom[20];
	u8 str[10];
	u8 str1[8];
	u8 s;
	str[0] = 'F';
	str[1] = 'S';
	str[2] = '1';
	str[3] = 0;
	if(flag) {  //È«¾ÖÔËÐÐ
		//·À×²¸Ë´¥·¢
		if(Portray_State[Din__Crashworthy_Pole].state == Portray_State[Din__Crashworthy_Pole].enaType)
			cmv.trouble_flag[2] = 1;
		else
			cmv.trouble_flag[2] = 0;

		for(i=2; i<5; i++) {  //¼ì²âÊÇ·ñÐèÒª¹Ø±Õ»úÆ÷
			if(cmv.trouble_flag[i])  {
				if(cmv.start_state) {  //Æô¶¯ºó¹Ø±Õ»úÆ÷
					cmv.start_state = 0;
					while(!Button_timer(Dout__Stop));
				}
				break;
			}
		}

		for(i=0; i<5; i++) {  //µ±¹ÊÕÏÊý¾Ý·¢Éú¸üÐÂ²Å·¢ËÍ
			if(trouble_copy[i] != cmv.trouble_flag[i]) {
				state = 1;
				break;
			}
		}


		if(state) {  //ÕûÀí¹ÊÕÏÊý¾Ý
			//	Voice_Transmit(9);  //²¥·ÅÓïÒô

			copyBuf(strCom, str);
			for(i=0; i<5; i++) {
				if(cmv.trouble_flag[i]) 
				{

					str1[0] = '_';
					str1[1] = 0x30 + i;
					str1[2] = 0;
					SendString_Jojnt(strCom, str1);
					copyBuf(strCom, OutputCombinedBuffer); 
					//s = i;
					//*str1 = s;
					//SendString_Jojnt(str, str1);
					//network_send_data[3] = network_send_data[3] + '_' + i;
				}
			}
			Voice_Transmit(9);  //²¥·ÅÓïÒô

			//OutputCombinedBuffer = str;
			//	RS485_SendString(str);
			network_send_data[3] = OutputCombinedBuffer;
		}

		if(state && !cmv.trouble[1]) {  //¿ÉÒÔ·¢ËÍ£¬Î´½ÓÊÕµ½ÏûÏ¢
			Voice_Transmit(9);  //²¥·ÅÓïÒô
			Car_Network_Send(3);  //·¢ËÍ¹ÊÕÏÊý¾Ý
			cmv.trouble[0] = 1;  //½ÓÊÕÊ¹ÄÜ´ò¿ª
		}
		else if(cmv.trouble[1]) {  //½ÓÊÕµ½Êý¾Ý
			state = 0;  //µ±Ç°¹ÊÕÏÊý¾ÝÒÑ·¢ËÍ
			cmv.trouble[0] = 0;  //½ÓÊÕÊ¹ÄÜ¹Ø±Õ
			cmv.trouble[1] = 0;  //Çå³ý½ÓÊÕµÄÊý¾Ý
			for(i=0; i<5; i++)  //¼ÇÂ¼ÒÑ·¢ËÍµÄÏûÏ¢
				trouble_copy[i] = cmv.trouble_flag[i];
		}
	}
	else {  //ÔËÓª×´Ì¬ÔËÐÐ
		//À¯Ë®ÒºÎ»´«¸ÐÆ÷´¥·¢
		if(Portray_State[Din__Wax_Fluid_Place].state == Portray_State[Din__Wax_Fluid_Place].enaType)
			cmv.trouble_flag[0] = 1;
		else
			cmv.trouble_flag[0] = 0;

		//ÅÝÄ­ÒºÎ»´«¸ÐÆ÷´¥·¢
		if(Portray_State[Din__Foam_Fluid_Place].state == Portray_State[Din__Foam_Fluid_Place].enaType)
			cmv.trouble_flag[1] =  1;
		else
			cmv.trouble_flag[1] = 0;

		//ÏÞ¸ß´¥·¢
		if(Portray_State[Din__Car_Restrict_Height].state == Portray_State[Din__Car_Restrict_Height].enaType)
			cmv.trouble_flag[3] = 1;
		else
			cmv.trouble_flag[3] = 0;

		//	µ¹·üÏÞÎ»´¥·¢ÔÝÊ±²»ÓÃÁ¬½Ó
		if(Portray_State[Din__Topple].state == Portray_State[Din__Topple].enaType)
			cmv.trouble_flag[4] = 1;
		else
			cmv.trouble_flag[4] = 0;
	}
}


/**
* @brief  £ºÉ¨Âë¼ì²â
* @param  £ºÎÞ
* @retval : ÎÞ
*/
static u8 ScanCodeInspect(void)
{
	static u8 procedure_flag = 0;  //¼ì²â²½Öè
	static u8 timer_flag = 0;      //É¨Âë·ÃÎÊ·þÎñÆ÷Ê±Ðò

	switch(procedure_flag)
	{
	case 0:
		{
			//µÚÒ»´Î´æÔÚ´¥·¢

			if(Portray_State[Din__Car_Exist].state == Portray_State[Din__Car_Exist].enaType)
				procedure_flag = 1;
		}break;

	case 1:
		{
			//³µÌå¹ýÁ¿¼ì²â

			Portray_State[Dout__Light1].state = 1 - Portray_State[Dout__Light1].enaType;
			Portray_State[Dout__Light2].state = 1 - Portray_State[Dout__Light2].enaType;
			Portray_State[Dout__Light3].state = 1 - Portray_State[Dout__Light3].enaType;
			if(Portray_State[Din__Car_Excess].state == Portray_State[Din__Car_Excess].enaType) {
				Portray_State[Dout__Light1].state = Portray_State[Dout__Light1].enaType;
				Voice_Transmit(3);  //²¥·ÅÓïÒôµ¹³µ
			}
			//³µÌå´æÔÚ¼ì²â
			else if(Portray_State[Din__Car_Exist].state == Portray_State[Din__Car_Exist].enaType) {
				Portray_State[Dout__Light2].state = Portray_State[Dout__Light2].enaType;
				Voice_Transmit(2);  //²¥·ÅÓïÒôÍ£³µ

				Timer_Pulse(1, 1);//¿ªÆôÊ±¼ä¼ì²â
				procedure_flag = 2;
			}
			else {
				Voice_Transmit(5);  //²¥·ÅÓïÒôÃ»ÓÐÍ£ºÃ
				Portray_State[Dout__Light3].state = Portray_State[Dout__Light3].enaType;
			}
		}break;

	case 2:
		{
			//³µÌå¹ýÁ¿´¥·¢»òÕß´æÔÚÎ´´¥·¢
			if(Portray_State[Din__Car_Excess].state == Portray_State[Din__Car_Excess].enaType  ||
				Portray_State[Din__Car_Exist].state != Portray_State[Din__Car_Exist].enaType)
			{
				procedure_flag = 1;
			}
			else if(Timer_Pulse(0, 1) == 5)
			{
				procedure_flag = 3;

				if(timer_flag)
					procedure_flag = 4; //Ö±½ÓÏ´³µ
				else
					Voice_Transmit(4);  //²¥·ÅÓïÒôÉ¨ÂëÌáÐÑ
			}
		}break;

	case 3:
		{
			//³µÌå¹ýÁ¿´¥·¢»òÕß´æÔÚÎ´´¥·¢
			if(Portray_State[Din__Car_Excess].state == Portray_State[Din__Car_Excess].enaType  ||
				Portray_State[Din__Car_Exist].state != Portray_State[Din__Car_Exist].enaType)
			{
				procedure_flag = 1;
			}

			cmv.car_payment_flag[1] = 1;//Ê¹ÄÜÉ¨Âë½ÓÊÕ
			//	Car_Network_Send(5);  //Ñ¯ÎÊÉ¨ÂëÊÇ·ñ³É¹¦

			if(cmv.car_payment_flag[2]) {  //É¨Âë³É¹¦
				cmv.car_payment_flag[0] = 0;
				cmv.car_payment_flag[1] = 0;
				cmv.car_payment_flag[2] = 0;

				cmv.run_service[0] = 2;  //É¨Âë¼ÇÂ¼£¬Ï´³µ±êÖ¾£¬Èç¹ûÍ£»ú±ØÐëÏ´ÍêÕâÒ»´Î
				timer_flag = 1;
				procedure_flag = 4;
			}
		}break;

	case 4:
		{
			timer_flag = 0;
			Voice_Transmit(6);  //²¥·ÅÓïÒôÌáÊ¾¿ªÊ¼Ï´³µ

			procedure_flag = 5;
		}break;

	case 5:
		{
			while(!Button_timer(Dout__Start));  //Æô¶¯Ï´³µ
			procedure_flag = 6;
		}break;

	case 6:
		{
			if(Portray_State[Din__Car_Init].state != Portray_State[Din__Car_Init].enaType)
			{
				procedure_flag = 0;
				cmv.start_state = 1;
				return 1;
			}
			else 
				procedure_flag = 5;
		}break;
	}

	return 0;
}


/**
* @brief  £ºÓïÒô²¥±¨
* @param  £ºÎÞ
* @retval : ÎÞ
*/
static void Voice_Transmit(u8 kind)
{	
	/*
	static u8 voice_flag = 0;
	u8 i1=0, i2=0;

	if(kind != voice_flag)
	{
	voice_flag = kind;

	i1 = 0x30+kind;
	i2 = 0xD7+kind;

	Usart_SendByte(USART3, 0xAA);Usart_SendByte(USART3, 0x08);Usart_SendByte(USART3, 0x0B);Usart_SendByte(USART3, 0x01);Usart_SendByte(USART3, 0x2F);
	Usart_SendByte(USART3, 0x30);Usart_SendByte(USART3, 0x30);Usart_SendByte(USART3, 0x30);Usart_SendByte(USART3, 0x30);Usart_SendByte(USART3, i1);
	Usart_SendByte(USART3, 0x2A);Usart_SendByte(USART3, 0x4D);Usart_SendByte(USART3, 0x50);Usart_SendByte(USART3, 0x33);Usart_SendByte(USART3, i2);
	}*/
	static u8 voice_flag = 0;

	if(kind != voice_flag)
	{
		voice_flag = kind;
		Usart_SendByte(USART3, 0x7E);
		Usart_SendByte(USART3, 0x05);
		Usart_SendByte(USART3, 0x41);
		Usart_SendByte(USART3, 0x00);
		Usart_SendByte(USART3, kind);

		switch(kind)
		{

		case 1:
			Usart_SendByte(USART3, 0x45);
			break;
		case 2:
			Usart_SendByte(USART3, 0x46);
			break;
		case 3:
			Usart_SendByte(USART3, 0x47);
			break;
		case 4:
			Usart_SendByte(USART3, 0x40);
			break;
		case 5:
			Usart_SendByte(USART3, 0x41);
			break;
		case 6:
			Usart_SendByte(USART3, 0x42);
			break;
		case 7:
			Usart_SendByte(USART3, 0x43);
			break;
		case 8:
			Usart_SendByte(USART3, 0x4C);
			break;
		}

		Usart_SendByte(USART3, 0xEF);
	}
}


/**
* @brief  £ºÏ´³µ¼ì²â
* @param  £ºÎÞ
* @retval : ÎÞ
*/
int lastVoiceState = 0;
int stateFanShack = 0;
static u8 Start(void)
{
	switch(cmv.wind_flag[1])
	{
	case 0:  if(Portray_State[Din__Fan].state == 1)  cmv.wind_flag[1]++;  break; 
	case 1:
		{
			stateFanShack = 0;
			if(cmv.wind_flag[0]) {  //ÅÐ¶ÏÄÜ·ñ´µ·ç
				cmv.wind_flag[1]++;
			}
			else {
				while(!Button_timer(Dout__Stop));
				cmv.wind_flag[1] = 0; 
				return 1;
			}
		}break;

	case 2: 
		if(stateFanShack == 0 && Portray_State[Din__Fan].state == 1 && Portray_State[Din__Car_Exist].state == 1 - Portray_State[Din__Car_Exist].enaType)
		{
			stateFanShack = 1;
		}
		//	if(Portray_State[Din__Car_Init].state == Portray_State[Din__Car_Init].enaType)  
		if(Portray_State[Din__Fan].state == 0 && stateFanShack)
		{
			cmv.wind_flag[1]++;

			while(!Button_timer(Dout__Stop));
		}					break;

	case 3: cmv.wind_flag[1] = 0; return 1;  break;
	}
	//if(cmv.wind_flag[1] != lastVoiceState)
	//{
	//	cmv.wind_flag[1] % 2 ? Voice_Transmit(1) : Voice_Transmit(2);  //²¥·ÅÓïÒôÌáÊ¾¿ªÊ¼Ï´³µ
	//}
	//lastVoiceState = cmv.wind_flag[1];
	return 0;
}


/**
* @brief  £º°´Å¥Êä³öÊ±¼ä¼ì²â
* @param  £ºÎÞ
* @retval : ÎÞ
*/
static u8 Button_timer(u8 kind)
{
	static u8 kind_flag[3] = {0};
	Control_InputOutput();//¿ØÖÆ¹ÜÀíÊäÈëÊä³ö

	switch(kind)
	{
	case Dout__Start:
		{
			if(kind_flag[0] == 1  &&  Timer_Pulse(0, 2) == 3)
			{
				Portray_State[Dout__Start].state = !Portray_State[Dout__Start].enaType;

				kind_flag[0] = 0;
				return 1;
			}
			else if(kind_flag[0] == 0)
			{
				Portray_State[Dout__Start].state = Portray_State[Dout__Start].enaType;
				kind_flag[0] = 1;
				Timer_Pulse(1, 2);//¿ªÆôÊ±¼ä¼ì²â
			}
		}break;

	case Dout__Stop:
		{
			if(kind_flag[1] == 1  &&  Timer_Pulse(0, 3) == 3)
			{
				Portray_State[Dout__Stop].state = !Portray_State[Dout__Stop].enaType;

				kind_flag[1] = 0;
				return 1;
			}
			else if(kind_flag[1] == 0)
			{
				Portray_State[Dout__Stop].state = Portray_State[Dout__Stop].enaType;
				kind_flag[1] = 1;
				Timer_Pulse(1, 3);//¿ªÆôÊ±¼ä¼ì²â
			}
		}break;

	case Dout__Scram:
		{
			if(kind_flag[2] == 1  &&  Timer_Pulse(0, 4) == 3)
			{
				Portray_State[Dout__Scram].state = !Portray_State[Dout__Scram].enaType;

				kind_flag[2] = 0;
				return 1;
			}
			else if(kind_flag[2] == 0)
			{
				Portray_State[Dout__Scram].state = Portray_State[Dout__Scram].enaType;
				kind_flag[2] = 1;
				Timer_Pulse(1, 4);//¿ªÆôÊ±¼ä¼ì²â
			}
		}break;
	}

	return 0;
}


u16 Init_ADC(u8 id)//PA0   ADC0
{ 
	ADC_InitTypeDef   ADC_InitStructure;   
	GPIO_InitTypeDef  GPIO_InitStructure;		
	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_0|GPIO_Pin_1| GPIO_Pin_2|GPIO_Pin_3      ;//ÉèÖÃ	ADC ÊäÈëÍ¨µÀa123	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	                                     //ÉèÖÃGPIOÄ£Ê½ Ä£ÄâÁ¿ÊäÈë
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA| /*RCC_APB2Periph_GPIOB|*/ RCC_APB2Periph_GPIOC, ENABLE);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	               //1¶ÀÁ¢µÄ×ª»»Ä£Ê½
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;		                   //2¿ªÆôÉ¨ÃèÄ£Ê½
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	                //3¿ªÆôÁ¬Ðø×ª»»Ä£Ê½
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //4ADCÍâ²¿¿ª¹Ø£¬¹Ø±Õ×´Ì¬
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;              //5¶ÔÆë·½Ê½,ADCÎª12Î»ÖÐ£¬ÓÒ¶ÔÆë·½Ê½
	ADC_InitStructure.ADC_NbrOfChannel =1;                              //6¿ªÆôÍ¨µÀÊý£¬1¸ö 
	ADC_Init(ADC1, &ADC_InitStructure);                                         
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_55Cycles5); //pa1 pin11
	ADC_DMACmd(ADC1,ENABLE);	                    //9ADCÃüÁî£¬Ê¹ÄÜ
	ADC_Cmd(ADC1, ENABLE);	                      //10¿ªÆôADC1	 
	ADC_ResetCalibration(ADC1);                   //11ÖØÐÂÐ£×¼
	while(ADC_GetResetCalibrationStatus(ADC1));   //12µÈ´ýÖØÐÂÐ£×¼Íê³É
	ADC_StartCalibration(ADC1);                   //13¿ªÊ¼Ð£×¼
	while(ADC_GetCalibrationStatus(ADC1));        //14µÈ´ýÐ£×¼Íê³É
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);       //15Á¬Ðø×ª»»¿ªÊ¼£¬ADCÍ¨¹ýDMA·½Ê½²»¶ÏµÄ¸üÐÂRAMÇø¡£	 
	return 0;

}

u16 Init_DMA_Ena(u8 id)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		                           //1Ê¹ÄÜDMAÊ±ÖÓ  	  
	DMA_DeInit(DMA1_Channel1);                                                         //2¿ªÆôDMA1µÄµÚÒ»Í¨µÀ
	DMA_InitStructure.DMA_PeripheralBaseAddr =DR_ADDRESS;	                           //3DMA¶ÔÓ¦µÄÍâÉè»ùµØÖ·((uint32_t)0x4001244C)       
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;                   //4ÄÚ´æ´æ´¢»ùµØÖ·
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	                               //5DMAµÄ×ª»»Ä£Ê½ÎªSRCÄ£Ê½£¬ÓÉÍâÉè°áÒÆµ½ÄÚ´æ		
	DMA_InitStructure.DMA_BufferSize = 10;                                            //6DMA»º´æ´óÐ¡£¬1¸ö
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                   //7½ÓÊÕÒ»´ÎÊý¾Ýºó£¬Éè±¸µØÖ·½ûÖ¹ºóÒÆ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//	--------------------   //8¹Ø±Õ½ÓÊÕÒ»´ÎÊý¾Ýºó£¬Ä¿±êÄÚ´æµØÖ·ºóÒÆ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;;       //9¶¨ÒåÍâÉèÊý¾Ý¿í¶ÈÎª16Î»
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;                //10DMA°áÒÆÊý¾Ý³ß´ç£¬HalfWord¾ÍÊÇÎª16Î»
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                                    //11×ª»»Ä£Ê½£¬Ñ­»·»º´æÄ£Ê½¡£
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;                                //12DMAÓÅÏÈ¼¶¸ß
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;              	                       //13M2MÄ£Ê½½ûÓÃ	 
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);	                                   //14 
	DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);                                      //
	DMA_ClearFlag(DMA1_FLAG_TC1);                                                      //
	DMA_Cmd(DMA1_Channel1, ENABLE);	                                                   //15
	return 0;
}



int GetStandardProc()
{
	u8 flagGoInto;

	int64_t sumAll;
	u16 gotInfo, gotInfoRem;//got into buffer
	int countGot;
	int totalCount;
	int totalSum;
	totalSum = 0;
	totalCount = 0;
	while(1)
	{
		switch(flagGoInto)
		{
		case 0:
			sumAll = 0; 
			flagGoInto ++;
			countGot = 0;

			break;
		case 1: 
			sumAll += ADC_ConvertedValue[0];
			countGot ++; 

			if(Timer_Pulse(0, 1) > 5 )
				flagGoInto ++;

			break;
		case 2:
			gotInfo = sumAll / countGot;
			totalSum += gotInfo;
			totalCount ++;

			Timer_Pulse(1, 1);
			flagGoInto = 0;
			break;
		default:
			flagGoInto = 0;
			break;
		}
		if(totalCount == 10)
			break;
	}
	return totalSum / 10;
}
/*********************************************END OF FILE**********************/



void TestingInfoAnalyze(u16 standard, u16 targetLevel, u16 SideDelta)
{

	u8 flagGoInto;
	int64_t sumAll;
	u16 gotInfo, gotInfoRem;//got into buffer
	int countGot;
	int totalCount;
	int totalSum;
	totalSum = 0;
	totalCount = 0;
	while(1)
	{
		switch(flagGoInto)
		{
		case 0:
			sumAll = 0; 
			flagGoInto ++;
			countGot = 0;

			break;
		case 1: 
			sumAll += ADC_ConvertedValue[0];
			countGot ++; 

			if(Timer_Pulse(0, 1) > 5 )
				flagGoInto ++;

			break;
		case 2:
			gotInfo = sumAll / countGot;

			PBout(0) = 1;
			PBout(1) = 1;
			if(gotInfo > targetLevel + standard + SideDelta)//too much
			{
				gotInfo = gotInfo - (targetLevel + standard + SideDelta);

				Usart_SendByte(USART3, 0xBB); 
				Usart_SendByte(USART3, 0x00); 
				Usart_SendByte(USART3, ((gotInfo) >> 0) & 0xff);
				Usart_SendByte(USART3, ((gotInfo) >> 8) & 0xff);
				Usart_SendByte(USART3, 0x0D); 
			}
			else if(gotInfo < targetLevel + standard - SideDelta)//untouched
			{
				gotInfo = (targetLevel + standard + SideDelta) - gotInfo;
				Usart_SendByte(USART3, 0xBB);
				Usart_SendByte(USART3, 0x01);
				Usart_SendByte(USART3, ((gotInfo) >> 0) & 0xff);
				Usart_SendByte(USART3, ((gotInfo) >> 8) & 0xff);
				Usart_SendByte(USART3, 0x0D);
			}
			else
			{
				gotInfo = (targetLevel + standard + SideDelta) - gotInfo;
				Usart_SendByte(USART3, 0xBB); 
				Usart_SendByte(USART3, 0x01); 
				Usart_SendByte(USART3, 0x00);
				Usart_SendByte(USART3, 0x00);
				Usart_SendByte(USART3, 0x0D); 
			}

			Usart_SendByte(USART3, 0x00); 
			PBout(0) = 0;
			PBout(1) = 0;
			Timer_Pulse(1, 1);
			flagGoInto = 0;
			break;
		default:
			flagGoInto = 0;
			break;
		} 
		if(flagCurrentTestingState != 2)
			break;
	}
}

