#include "stm8l152c6.h"
#include "math.h"
#include "stdbool.h" 
                    

_Bool MUL_A0 @ PB_ODR : 4;        
_Bool MUL_A1 @ PB_ODR : 5;        
_Bool MUL_A2 @ PB_ODR : 6;        
_Bool MUL_A3 @ PB_ODR : 7;        

_Bool DUL @ FLASH_IAPSR: 3; 

_Bool HSI_RDY @ CLK_ICKCR : 1;
      
_Bool START_CONV @ ADC1_CR1 : 1;

_Bool FAULT_PIN @ PB_ODR : 1;

#define NO_BYETES_HDP 50

_Bool RS485_ENABLE @ PC_ODR     : 7; 
_Bool EMPTY        @ USART1_SR  : 7;
_Bool RX_enable    @ USART1_CR2 : 2;
_Bool TX_enable    @ USART1_CR2 : 3;
_Bool TX           @ USART1_CR2 : 6;

_Bool PE   @ USART1_SR : 0;
_Bool FE   @ USART1_SR : 1;
_Bool NF   @ USART1_SR : 2;
_Bool OR   @ USART1_SR : 3;
_Bool RXNE @ USART1_SR : 5;

#define RATIO_LMT_L 0x00001220
#define RATIO_LMT_H 0x00001221

#define RATIO_LMT_L2 0x00001202
#define RATIO_LMT_H2 0x00001203

#define RATIO_LMT_L3 0x00001204
#define RATIO_LMT_H3 0x00001205

#define RATIO_LMT_CVT2L 0x00001206
#define RATIO_LMT_CVT2H 0x00001207

#define RATIO_LMT_TXR1L 0x00001210
#define RATIO_LMT_TXR1H 0x00001211

#define RATIO_LMT_TXR2L 0x0000120C
#define RATIO_LMT_TXR2H 0x0000120D


#define RATIO_LMT_TXR3L 0x00001212
#define RATIO_LMT_TXR3H 0x00001213

#define RATIO_LMT_TXR4L 0x00001214
#define RATIO_LMT_TXR4H 0x00001215

#define RATIO_LMT_TXR5L 0x00001216
#define RATIO_LMT_TXR5H 0x00001217

#define RATIO_LMT_TXR6L 0x00001218
#define RATIO_LMT_TXR6H 0x00001219

#define RATIO_LMT_TXR7L 0x0000121A
#define RATIO_LMT_TXR7H 0x0000121B

#define RATIO_LMT_TXR8L 0x0000121C
#define RATIO_LMT_TXR8H 0x0000121D


void LCD_CLEAR();

void EEPROM_Init(void);
void data_write(unsigned int address,unsigned char c);     
unsigned char data_read(unsigned int address);


unsigned char keycntr,cnt_KEY,max_cnt;
unsigned char flag0;
unsigned char flag1;
unsigned char flg_set1,flg_set;
unsigned int cntr,cntr0,mux_cntr,cnt;
unsigned char key,key_deb2,key_deb1,key_deb0,disp_cntr;

bool disp_flag;

unsigned int DISP,calib_lmt;

unsigned long int INV1_CNT,INV2_CNT;       //TXR4_V
unsigned long int CVT1_CNT,CVT2_CNT;

unsigned long int TXR4_CNT,TXR3_CNT,TXR2_CNT,TXR1_CNT;
unsigned long int TXR8_CNT,TXR7_CNT,TXR6_CNT,TXR5_CNT;

unsigned int TXR4_V,TXR3_V,TXR2_V,TXR1_V,CVT1_V,CVT2_V;
unsigned int TXR8_V,TXR7_V,TXR6_V,TXR5_V,INV1_V,INV2_V;

unsigned int cnt1,ratio_lmt,cnt2,cnt3,val1,val2,val;

unsigned int ratio_lmt1,ratio_lmt2,ratio_lmt_cvt2,ratio_lmt_t1,ratio_lmt_txr2;

unsigned int ratio_lmt_t3,ratio_lmt_t4,ratio_lmt_t5,ratio_lmt_t6,ratio_lmt_t7;
unsigned int ratio_lmt_t8;

unsigned long int RY_Adc_Cnt_01;

bool s_set1_flag;

////////////////////////////calib_val////////////////////

///////////////////////////////////////////////////////////

/***************************************************************/

//unsigned char thousands_seg,hundreds_seg,tens_seg,units_seg; 
unsigned char cntr_disp=0,dc_cntr,counter;
unsigned char fault=0,disp=0; //danzer variables dont delete
unsigned char flag12,flag10,flag11,checksum,data11; 
unsigned char c_tx_counter1,c_rx_counter1,computer,vlt1,vlt2;
unsigned char rdata[10],sdata[52];

unsigned int cou1,data1,adc_values[13],txdata,disp_cntr1,disp_cntr11;  //adc_values[12]

unsigned long int dc_cnt,dc_cnt1,dc_cnt2,dc_cnt3;
unsigned long int dc_cnt_zero,dc_cnt_one,dc_cnt4,dc_cnt5,dc_cnt6,dc_cnt7,dc_cnt8,dc_cnt9;
unsigned long int dc_cnt10,dc_cnt11,dc_cnt12,dc_cnt13,dc_cnt14,dc_cnt15;

unsigned char chinni_cntr1,chinni_cntr,mux1;mux2;  // ac

bool V0,V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12,V13,V14,V15; 

unsigned char flag5;

unsigned int Op_Voltage_Cnt1,m_sec_disp,Op_Vol_AVG_CNT1;

unsigned long int RY_Adc_Cnt_02,Op_Voltage;
unsigned long int RY_Adc_Cnt_03,RY_Adc_Cnt_09,Op_Vol_AVG_CNT; 

//////////////////////////////////////////////////////////////////////////

unsigned int Op_Voltage_Cnt1_1,m_sec_disp_1; 
unsigned long int RY_Adc_Cnt_01_1,RY_Adc_Cnt_02_1,Op_Voltage_1;
unsigned long int RY_Adc_Cnt_03_1,RY_Adc_Cnt_09_1,Op_Vol_AVG_CNT_1,Op_Vol_AVG_CNT1_1; 

///////////////////////////////////////////////////////////////////////////////

unsigned int Op_Voltage_Cnt1_2,m_sec_disp_2; 
unsigned long int RY_Adc_Cnt_01_2,RY_Adc_Cnt_02_2,Op_Voltage_2;
unsigned long int RY_Adc_Cnt_03_2,RY_Adc_Cnt_09_2,Op_Vol_AVG_CNT_2,Op_Vol_AVG_CNT1_2; 

/////////////////////////////inverter current///////////////////////////////////


unsigned long int Inv1_sum_cnt,Inv1_sum_cnt1,Inv1_sum_cnt2,Inv1_avg_cnt,Inv1_avg_cnt1;
unsigned long int Inv1_square_cnt,Inv1_sample_cnt1;
unsigned int Inv1_current,Inv1_curr_divider,Inv1_sample_cnt; //Inv1_current;


unsigned long int Inv2_sum_cnt,Inv2_sum_cnt1,Inv2_sum_cnt2,Inv2_avg_cnt,Inv2_avg_cnt1;
unsigned long int Inv2_square_cnt,Inv2_sample_cnt1;
unsigned int Inv2_current,Inv2_curr_divider,Inv2_sample_cnt; //Inv1_current;

///////////////////////////////// cvt1 current//////////////////////////////

unsigned long int Cvt1_sum_cnt,Cvt1_sum_cnt1,Cvt1_sum_cnt2,Cvt1_avg_cnt,Cvt1_avg_cnt1;
unsigned long int Cvt1_square_cnt,Cvt1_sample_cnt1;
unsigned int Cvt1_current,Cvt1_curr_divider,Cvt1_sample_cnt;

///////////////////////////////// cvt2 current//////////////////////////////

unsigned long int Cvt2_sum_cnt,Cvt2_sum_cnt1,Cvt2_sum_cnt2,Cvt2_avg_cnt,Cvt2_avg_cnt1;
unsigned long int Cvt2_square_cnt,Cvt2_sample_cnt1;
unsigned int Cvt2_current,Cvt2_curr_divider,Cvt2_sample_cnt;

///////////////////////////////// txr1 current//////////////////////////////

unsigned long int Txr1_sum_cnt,Txr1_sum_cnt1,Txr1_sum_cnt2,Txr1_avg_cnt,Txr1_avg_cnt1;
unsigned long int Txr1_square_cnt,Txr1_sample_cnt1;
unsigned int Txr1_current,Txr1_curr_divider,Txr1_sample_cnt;


///////////////////////////////// txr2 current//////////////////////////////

unsigned long int Txr2_sum_cnt,Txr2_sum_cnt1,Txr2_sum_cnt2,Txr2_avg_cnt,Txr2_avg_cnt1;
unsigned long int Txr2_square_cnt,Txr2_sample_cnt1;
unsigned int Txr2_current,Txr2_curr_divider,Txr2_sample_cnt;

///////////////////////////////// txr3 current//////////////////////////////

unsigned long int Txr3_sum_cnt,Txr3_sum_cnt1,Txr3_sum_cnt2,Txr3_avg_cnt,Txr3_avg_cnt1;
unsigned long int Txr3_square_cnt,Txr3_sample_cnt1;
unsigned int Txr3_current,Txr3_curr_divider,Txr3_sample_cnt;

///////////////////////////////// txr4 current//////////////////////////////

unsigned long int Txr4_sum_cnt,Txr4_sum_cnt1,Txr4_sum_cnt2,Txr4_avg_cnt,Txr4_avg_cnt1;
unsigned long int Txr4_square_cnt,Txr4_sample_cnt1;
unsigned int Txr4_current,Txr4_curr_divider,Txr4_sample_cnt;

///////////////////////////////// txr5 current//////////////////////////////

unsigned long int Txr5_sum_cnt,Txr5_sum_cnt1,Txr5_sum_cnt2,Txr5_avg_cnt,Txr5_avg_cnt1;
unsigned long int Txr5_square_cnt,Txr5_sample_cnt1;
unsigned int Txr5_current,Txr5_curr_divider,Txr5_sample_cnt;

//////////////////////////////// txr6 current//////////////////////////////

unsigned long int Txr6_sum_cnt,Txr6_sum_cnt1,Txr6_sum_cnt2,Txr6_avg_cnt,Txr6_avg_cnt1;
unsigned long int Txr6_square_cnt,Txr6_sample_cnt1;
unsigned int Txr6_current,Txr6_curr_divider,Txr6_sample_cnt;

//////////////////////////////// txr7 current//////////////////////////////

unsigned long int Txr7_sum_cnt,Txr7_sum_cnt1,Txr7_sum_cnt2,Txr7_avg_cnt,Txr7_avg_cnt1;
unsigned long int Txr7_square_cnt,Txr7_sample_cnt1;
unsigned int Txr7_current,Txr7_curr_divider,Txr7_sample_cnt;

//////////////////////////////// txr7 current//////////////////////////////

unsigned long int Txr8_sum_cnt,Txr8_sum_cnt1,Txr8_sum_cnt2,Txr8_avg_cnt,Txr8_avg_cnt1;
unsigned long int Txr8_square_cnt,Txr8_sample_cnt1;
unsigned int Txr8_current,Txr8_curr_divider,Txr8_sample_cnt;

///////////////////////Communication variables/////////////////////////////////////////////

unsigned int Comm_Inv1_vlt,Comm_Inv2_vlt,Comm_Cvt1_vlt,Comm_Cvt2_vlt;
unsigned int Comm_Txr1_vlt,Comm_Txr2_vlt,Comm_Txr3_vlt,Comm_Txr4_vlt;
unsigned int Comm_Txr5_vlt,Comm_Txr6_vlt,Comm_Txr7_vlt,Comm_Txr8_vlt;

unsigned int Comm_Inv1_current,Comm_Inv2_current,Comm_Cvt1_current,Comm_Cvt2_current;
unsigned int Comm_Txr1_current,Comm_Txr2_current,Comm_Txr3_current,Comm_Txr4_current;
unsigned int Comm_Txr5_current,Comm_Txr6_current,Comm_Txr7_current,Comm_Txr8_current;

/////////////////////////////////////////////////////////////////////////////////////////

unsigned int ratio_lmt_inv1,ratio_lmt_inv2,ratio_lmt2_cvt1,ratio_lmt_cvt22;
unsigned int ratio_lmt_t11,ratio_lmt_txr22,ratio_lmt_t33,ratio_lmt_t44;
unsigned int ratio_lmt_t55,ratio_lmt_t66,ratio_lmt_t77,ratio_lmt_t88;


	

#define LCD PE_ODR


	  
void ac_calc(void)
{
	  
	
  /////////////////////AC Output Voltage Measurement Calc////////////////////	
	
	  Inv1_sum_cnt=Inv1_square_cnt;	
		Inv1_sum_cnt1=(unsigned long int)(Inv1_sum_cnt/m_sec_disp);
	//	Inv2_sum_cnt1=1000;
		Inv1_avg_cnt=(unsigned long int)(sqrt((double)Inv1_sum_cnt1));
		Inv1_avg_cnt1=(unsigned int)Inv1_avg_cnt;
		Inv1_current=(unsigned int)((Inv1_avg_cnt*90)/1360);  //1019	

    Comm_Inv1_current=Inv1_current;
	
		Inv1_square_cnt=0; //m_sec_disp=0;
	
	///////////////////////////////////////////////////////
	
	  Inv2_sum_cnt=Inv2_square_cnt;	
		Inv2_sum_cnt1=(unsigned long int)(Inv2_sum_cnt/m_sec_disp);
	//	Inv2_sum_cnt1=1000;
		Inv2_avg_cnt=(unsigned long int)(sqrt((double)Inv2_sum_cnt1));
		Inv2_avg_cnt1=(unsigned int)Inv2_avg_cnt;
		Inv2_current=(unsigned int)((Inv2_avg_cnt*90)/1360);  //1019	

    Comm_Inv2_current=Inv2_current;
	
		Inv2_square_cnt=0; //m_sec_disp=0;
		
		//////////////////////////////////////////
		
		Cvt1_sum_cnt=Cvt1_square_cnt;	
		Cvt1_sum_cnt1=(unsigned long int)(Cvt1_sum_cnt/m_sec_disp);
	//	Inv2_sum_cnt1=1000;
		Cvt1_avg_cnt=(unsigned long int)(sqrt((double)Cvt1_sum_cnt1));
		Cvt1_avg_cnt1=(unsigned int)Cvt1_avg_cnt;
		Cvt1_current=(unsigned int)((Cvt1_avg_cnt*90)/1360);  //1019	

    Comm_Cvt1_current=Cvt1_current;
	
		Cvt1_square_cnt=0;  //m_sec_disp=0;		
		
		/////////////////////////////////////////////////
		
		Cvt2_sum_cnt=Cvt2_square_cnt;	
		Cvt2_sum_cnt1=(unsigned long int)(Cvt2_sum_cnt/m_sec_disp);
	//	Inv2_sum_cnt1=1000;
		Cvt2_avg_cnt=(unsigned long int)(sqrt((double)Cvt2_sum_cnt1));
		Cvt2_avg_cnt1=(unsigned int)Cvt2_avg_cnt;
		Cvt2_current=(unsigned int)((Cvt2_avg_cnt*90)/1360);  //1019	

    Comm_Cvt2_current=Cvt2_current;
	
		Cvt2_square_cnt=0;  //m_sec_disp=0;
		
		
		/////////////////////////// txr1////////////////////////////////
		
		Txr1_sum_cnt=Txr1_square_cnt;	
		Txr1_sum_cnt1=(unsigned long int)(Txr1_sum_cnt/m_sec_disp);
	//	Inv2_sum_cnt1=1000;
		Txr1_avg_cnt=(unsigned long int)(sqrt((double)Txr1_sum_cnt1));
		Txr1_avg_cnt1=(unsigned int)Txr1_avg_cnt;
		Txr1_current=(unsigned int)((Txr1_avg_cnt*90)/1360);  //1019		
		
		Comm_Txr1_current=Txr1_current;
	
		Txr1_square_cnt=0;  //m_sec_disp=0;
		
		
		
		/////////////////////////// txr2////////////////////////////////
		
		Txr2_sum_cnt=Txr2_square_cnt;	
		Txr2_sum_cnt1=(unsigned long int)(Txr2_sum_cnt/m_sec_disp);
	//	Inv2_sum_cnt1=1000;
		Txr2_avg_cnt=(unsigned long int)(sqrt((double)Txr2_sum_cnt1));
		Txr2_avg_cnt1=(unsigned int)Txr2_avg_cnt;
		Txr2_current=(unsigned int)((Txr2_avg_cnt*90)/1360);  //1019		
		
		Comm_Txr2_current=Txr2_current;
	
		Txr2_square_cnt=0;  //m_sec_disp=0;
		
		/////////////////////////// txr3////////////////////////////////
		
		Txr3_sum_cnt=Txr3_square_cnt;	
		Txr3_sum_cnt1=(unsigned long int)(Txr3_sum_cnt/m_sec_disp);
	//	Inv2_sum_cnt1=1000;
		Txr3_avg_cnt=(unsigned long int)(sqrt((double)Txr3_sum_cnt1));
		Txr3_avg_cnt1=(unsigned int)Txr3_avg_cnt;
		Txr3_current=(unsigned int)((Txr3_avg_cnt*90)/1360);  //1019		
		
		Comm_Txr3_current=Txr3_current;
	
		Txr3_square_cnt=0;  //m_sec_disp=0;
		
		/////////////////////////// txr4////////////////////////////////
		
		Txr4_sum_cnt=Txr4_square_cnt;	
		Txr4_sum_cnt1=(unsigned long int)(Txr4_sum_cnt/m_sec_disp);
	//	Inv2_sum_cnt1=1000;
		Txr4_avg_cnt=(unsigned long int)(sqrt((double)Txr4_sum_cnt1));
		Txr4_avg_cnt1=(unsigned int)Txr4_avg_cnt;
		Txr4_current=(unsigned int)((Txr4_avg_cnt*90)/1360);  //1019

    Comm_Txr4_current=Txr4_current;
	
		Txr4_square_cnt=0;  //m_sec_disp=0;
		
		/////////////////////////// txr5////////////////////////////////
		
		Txr5_sum_cnt=Txr5_square_cnt;	
		Txr5_sum_cnt1=(unsigned long int)(Txr5_sum_cnt/m_sec_disp);
	//	Inv2_sum_cnt1=1000;
		Txr5_avg_cnt=(unsigned long int)(sqrt((double)Txr5_sum_cnt1));
		Txr5_avg_cnt1=(unsigned int)Txr5_avg_cnt;
		Txr5_current=(unsigned int)((Txr5_avg_cnt*90)/1360);  //1019		
		
		Comm_Txr5_current=Txr5_current;
	
		Txr5_square_cnt=0;  //m_sec_disp=0;
		
		/////////////////////////// txr6////////////////////////////////
		
		Txr6_sum_cnt=Txr6_square_cnt;	
		Txr6_sum_cnt1=(unsigned long int)(Txr6_sum_cnt/m_sec_disp);
	//	Inv2_sum_cnt1=1000;
		Txr6_avg_cnt=(unsigned long int)(sqrt((double)Txr6_sum_cnt1));
		Txr6_avg_cnt1=(unsigned int)Txr6_avg_cnt;
		Txr6_current=(unsigned int)((Txr6_avg_cnt*90)/1360);  //1019		
		
		Comm_Txr6_current=Txr6_current;
	
		Txr6_square_cnt=0;  //m_sec_disp=0;
		
		/////////////////////////// txr7////////////////////////////////
		
		Txr7_sum_cnt=Txr7_square_cnt;	
		Txr7_sum_cnt1=(unsigned long int)(Txr7_sum_cnt/m_sec_disp);
	//	Inv2_sum_cnt1=1000;
		Txr7_avg_cnt=(unsigned long int)(sqrt((double)Txr7_sum_cnt1));
		Txr7_avg_cnt1=(unsigned int)Txr7_avg_cnt;
		Txr7_current=(unsigned int)((Txr7_avg_cnt*90)/1360);  //1019		
		
		Comm_Txr7_current=Txr7_current;
	
		Txr7_square_cnt=0;  //m_sec_disp=0;
		
		/////////////////////////// txr8////////////////////////////////
		
		Txr8_sum_cnt=Txr8_square_cnt;	
		Txr8_sum_cnt1=(unsigned long int)(Txr8_sum_cnt/m_sec_disp);
	//	Inv2_sum_cnt1=1000;
		Txr8_avg_cnt=(unsigned long int)(sqrt((double)Txr8_sum_cnt1));
		Txr8_avg_cnt1=(unsigned int)Txr8_avg_cnt;
		Txr8_current=(unsigned int)((Txr8_avg_cnt*90)/1360);  //1019		
		
		Comm_Txr8_current=Txr8_current;
	
		Txr8_square_cnt=0;  //m_sec_disp=0;
		
				
		m_sec_disp=0;
		
		
		
			/* RY_Adc_Cnt_03=RY_Adc_Cnt_02;	
															 RY_Adc_Cnt_09=(unsigned long int)(RY_Adc_Cnt_03/m_sec_disp);
												
															 Op_Vol_AVG_CNT=(unsigned long int)(sqrt((double)RY_Adc_Cnt_09));
															 
															 Op_Vol_AVG_CNT1=(unsigned int)Op_Vol_AVG_CNT;
															 
															 Op_Voltage=(unsigned long int)((Op_Vol_AVG_CNT*300)/303);
															 
															 Op_Vol_AVG_CNT1=Op_Vol_AVG_CNT;
														
															 RY_Adc_Cnt_02=0;	m_sec_disp=0;  //1040  
															 
	*/
	
}	


void delay1(unsigned int D)
{
	unsigned int i;
	while(D)
	{
		for(i=0;i<100;i++);
		D--;
	}
	
	return;	
}

void cmd_lcd(unsigned char c)
{   
   unsigned char Temp=0;
     Temp=c;
		 LCD&=0XC0;		
     LCD|=(c>>4); //msb
		 delay1(1);
		 LCD|=(0x10); // EN HIGH
		 delay1(1);
		 LCD&=0xCF;  // EN LOW
		 LCD&=0XC0;   
		 LCD|=(Temp&0x0F); //lsb
		 delay1(1);
		 LCD|=(0x10); // EN HIGH
		 delay1(1);
		 LCD&=0xCF;  // EN LOW
  	 delay1(10);
		 
     return;
}  

void Lcd_Init(void)
{
		cmd_lcd(0x33);
		delay1(300);
		cmd_lcd(0x32);
    delay1(300);
    cmd_lcd(0x2C);
    cmd_lcd(0x0c);
    cmd_lcd(0x06);
    cmd_lcd(0x01); 
			
} 
 
void write_lcd(unsigned char c)
{   
   unsigned char Temp=0;
     Temp=c;
		 LCD&=0XC0;	
		 LCD|=0X20;	
     LCD|=(c>>4); //msb
		 delay1(1);
		 LCD|=(0x10); // Enable HIGH
		 delay1(1);
     LCD&=0xEF;// Enable LOW
		 LCD&=0XE0;
		 LCD|=(Temp&0x0F); //lsb 
		 delay1(1);
		 LCD|=(0x10);// Enable HIGH
		 delay1(1);
		 LCD&=0xEF;// Enable LOW
     delay1(10);	
	
	 return;
}  

void display_lcd(unsigned char *s)
{
	 while(*s) 
   write_lcd(*s++);
} 

void lcd_integer(unsigned int n)
{    
  unsigned char c[6],i=0;
		
    if(n==0)
     write_lcd('0');
    while(n>0)
    {
       c[i++]=((n%10)+48);
       n=(n/10);
    }
    while(i-->=1)
    write_lcd(c[i]);
		
	return;	
}

void longinteger_lcd(unsigned long int f_Num)
{   
    unsigned char c[11],i=0;         
     
    if(f_Num==0)
		write_lcd('0');
        
    while(f_Num>0)
    {
      c[i++]=(f_Num%10)+48;
      f_Num=f_Num/10;
    }       
    
    while(i-->=1)
    write_lcd(c[i]);
} 

void lcd_float(unsigned int n)
{
  unsigned long temp1;   
    temp1=n;
    n=(n/10);
    temp1=(temp1%10);
    lcd_integer(n);
    write_lcd('.');
    lcd_integer(temp1);
		
	return;	
}


unsigned int get_calibvalue(unsigned int loch,unsigned int locl)
{
	  val1=data_read(loch);
		delay1(10);
		val2=val1<<8;
		val2|=data_read(locl);
		delay1(10);

   return val2;
}


void read_calib(void)
{
	 ratio_lmt=get_calibvalue(RATIO_LMT_H,RATIO_LMT_L);
	 if(ratio_lmt<1000 || ratio_lmt>3000)
	 ratio_lmt=2300;
	 
	 ratio_lmt_inv1=ratio_lmt;
	 
	 ratio_lmt1=get_calibvalue(RATIO_LMT_H2,RATIO_LMT_L2);
	 if(ratio_lmt1<1000 || ratio_lmt1>3000)
	 ratio_lmt1=2300;
	 
	 ratio_lmt_inv2=ratio_lmt1;
	 
	 ratio_lmt2=get_calibvalue(RATIO_LMT_H3,RATIO_LMT_L3);
	 if(ratio_lmt2<1000 || ratio_lmt2>3000)
	 ratio_lmt2=2300;
	 
	 ratio_lmt2_cvt1=ratio_lmt2;
	 
	 ratio_lmt_cvt2=get_calibvalue(RATIO_LMT_CVT2H,RATIO_LMT_CVT2L);
	 if(ratio_lmt_cvt2<1000 || ratio_lmt_cvt2>3000)
	 ratio_lmt_cvt2=2300;
	 
	 ratio_lmt_cvt22=ratio_lmt_cvt2;
	 
	 ratio_lmt_t1=get_calibvalue(RATIO_LMT_TXR1H,RATIO_LMT_TXR1L);
	 if(ratio_lmt_t1<700 || ratio_lmt_t1>3000)
	 ratio_lmt_t1=900;
	 
	 ratio_lmt_t11=ratio_lmt_t1;
	 
	 ratio_lmt_txr2=get_calibvalue(RATIO_LMT_TXR2H,RATIO_LMT_TXR2L);
	 if(ratio_lmt_txr2<700 || ratio_lmt_txr2>3000)
	 ratio_lmt_txr2=900;
	 
	 ratio_lmt_txr22=ratio_lmt_txr2;

   ratio_lmt_t3=get_calibvalue(RATIO_LMT_TXR3H,RATIO_LMT_TXR3L);
	 if(ratio_lmt_t3<700 || ratio_lmt_t3>3000)
	 ratio_lmt_t3=900;
	 
	 ratio_lmt_t33=ratio_lmt_t3;
	 
	 ratio_lmt_t4=get_calibvalue(RATIO_LMT_TXR4H,RATIO_LMT_TXR4L);
	 if(ratio_lmt_t4<700 || ratio_lmt_t4>3000)
	 ratio_lmt_t4=900;
	 
	 ratio_lmt_t44=ratio_lmt_t4;
	 
	 ratio_lmt_t5=get_calibvalue(RATIO_LMT_TXR5H,RATIO_LMT_TXR5L);
	 if(ratio_lmt_t5<700 || ratio_lmt_t5>3000)
	 ratio_lmt_t5=900;
	 
	 ratio_lmt_t55=ratio_lmt_t5;
	 
	 ratio_lmt_t6=get_calibvalue(RATIO_LMT_TXR6H,RATIO_LMT_TXR6L);
	 if(ratio_lmt_t6<700 || ratio_lmt_t6>3000)
	 ratio_lmt_t6=900;
	 
	 ratio_lmt_t66=ratio_lmt_t6;
	 
	 ratio_lmt_t7=get_calibvalue(RATIO_LMT_TXR7H,RATIO_LMT_TXR7L);
	 if(ratio_lmt_t7<700 || ratio_lmt_t7>3000)
	 ratio_lmt_t7=900;
	 
	 ratio_lmt_t77=ratio_lmt_t7;
	 
	 ratio_lmt_t8=get_calibvalue(RATIO_LMT_TXR8H,RATIO_LMT_TXR8L);
	 if(ratio_lmt_t8<700 || ratio_lmt_t8>3000)
	 ratio_lmt_t8=900;
	 
	 ratio_lmt_t88=ratio_lmt_t8;
	 	 
	 delay1(10);
}

void EEPROM_Init(void)
{
	 FLASH_CR1=0X08;
	//  FLASH_CR2=0X40;
	 FLASH_DUKR=0XAE;
}	

void data_write(unsigned int address,unsigned char c)
{
   if(FLASH_DUKR==0XAE)
		FLASH_DUKR=0X56;
		
	while(DUL==0);
  (*((unsigned char*)address))=c;

	 DUL=0;
	 FLASH_DUKR=0XAE; 
}

unsigned char data_read(unsigned int address)
{	
	if(FLASH_DUKR==0XAE)
	 FLASH_DUKR=0X56;
		
	while(DUL==0);
	 val=*((unsigned char*)address);

	 DUL=0;
   FLASH_DUKR=0XAE;
	
	return val;
}

void LCD_CLEAR(void)
{
	cmd_lcd(0x80);
	display_lcd("        ");
	cmd_lcd(0xC0);
	display_lcd("        ");
}	

void calibration(void)
{          
  unsigned char c1;
	unsigned long int temp2;
  float temp1;	
	
	LCD_CLEAR();
	
	cmd_lcd(0x80);
	display_lcd("INV1_VC:");
	//cmd_lcd(0xC0);
	//display_lcd("INV1_V:");
	
  read_calib();	//cnt++;
							
	temp1=ratio_lmt;
	
disp_chk: 

 calib_lmt=temp1;

  INV1_V=(unsigned int)((INV1_CNT*230)/calib_lmt); 

  //cmd_lcd(0x88);
	//lcd_integer(INV1_V);

	cmd_lcd(0XC0);
	lcd_integer(INV1_V);
	write_lcd(' ');
	write_lcd(' ');
	
   if(key==0X05)  //if DOWN key press
  	 goto dec_shnt1;
	 else if(key==0X06)  //if UP key press
     goto inc_shnt1; //if SET key press
	 else if((key==0X03)&&(s_set1_flag==0))       
     goto s_set1;
	 else 
	   {goto disp_chk;}

inc_shnt1:
  temp1+=0.05;	
	//cnt2++;  
   if(temp1>3000)//1450)
   temp1=3000;//1450;
	 s_set1_flag=0;
	 goto disp_chk;

dec_shnt1:
  temp1-=0.05;	
	//cnt3++;
	 if(temp1<1000)
   temp1=1000;   
   s_set1_flag=0;  
   goto disp_chk;

 s_set1:
    temp2=temp1;
    c1=temp2;
	  data_write(RATIO_LMT_L,c1);
	  delay1(20);
	  c1=temp2>>8;
	  data_write(RATIO_LMT_H,c1);
	  delay1(20);
	  s_set1_flag=1;  //cnt1++;
	  
		while(key!=0x07);
		
		read_calib();
		
		
		//////////////////// INV2/////////////////////
		
		cmd_lcd(0x80);
	  display_lcd("INV2_VC:");
	
    read_calib();	//cnt++;
							
	  temp1=ratio_lmt1;
	
disp_chk_inv2: 

    calib_lmt=temp1;

    INV2_V=(unsigned long int)((INV2_CNT*230)/calib_lmt); 

  //cmd_lcd(0x88);
	//lcd_integer(INV1_V);

	cmd_lcd(0XC0);
	lcd_integer(INV2_V);
	write_lcd(' ');
	write_lcd(' ');
	
   if(key==0X05)  //if DOWN key press
  	 goto dec_shnt_inv2;
	 else if(key==0X06)  //if UP key press
     goto inc_shnt_inv2; //if SET key press
	 else if((key==0X03)&&(s_set1_flag==0))       
     goto s_set_inv2;
	 else 
	   {goto disp_chk_inv2;}

inc_shnt_inv2:
  temp1+=0.05;	
	//cnt2++;  
   if(temp1>3000)//1450)
   temp1=3000;//1450;
	 s_set1_flag=0;
	 goto disp_chk_inv2;

dec_shnt_inv2:
  temp1-=0.05;	
	//cnt3++;
	 if(temp1<1000)
   temp1=1000;   
   s_set1_flag=0;  
   goto disp_chk_inv2;

 s_set_inv2:
    temp2=temp1;
    c1=temp2;
	  data_write(RATIO_LMT_L2,c1);
	  delay1(20);
	  c1=temp2>>8;
	  data_write(RATIO_LMT_H2,c1);
	  delay1(20);
	  s_set1_flag=1;  //cnt1++;
	  while(key!=0x07);
		
		read_calib();
		
		//////////////////CVT1_V//////////////////////////
		
		cmd_lcd(0x80);
	  display_lcd("CVT1_VC:");
	
    read_calib();	//cnt++;
							
	  temp1=ratio_lmt2;
	
disp_chk_cvt1: 

    calib_lmt=temp1;

    CVT1_V=(unsigned long int)((CVT1_CNT*230)/calib_lmt); 

  //cmd_lcd(0x88);
	//lcd_integer(INV1_V);

	cmd_lcd(0XC0);
	lcd_integer(CVT1_V);
	write_lcd(' ');
	write_lcd(' ');
	
   if(key==0X05)  //if DOWN key press
  	 goto dec_shnt_cvt1;
	 else if(key==0X06)  //if UP key press
     goto inc_shnt_cvt1; //if SET key press
	 else if((key==0X03)&&(s_set1_flag==0))       
     goto s_set_cvt1;
	 else 
	   {goto disp_chk_cvt1;}

inc_shnt_cvt1:
  temp1+=0.05;	
	//cnt2++;  
   if(temp1>3000)//1450)
   temp1=3000;//1450;
	 s_set1_flag=0;
	 goto disp_chk_cvt1;

dec_shnt_cvt1:
  temp1-=0.05;	
	//cnt3++;
	 if(temp1<1000)
   temp1=1000;   
   s_set1_flag=0;  
   goto disp_chk_cvt1;

 s_set_cvt1:
    temp2=temp1;
    c1=temp2;
	  data_write(RATIO_LMT_L3,c1);
	  delay1(20);
	  c1=temp2>>8;
	  data_write(RATIO_LMT_H3,c1);
	  delay1(20);
	  s_set1_flag=1;  //cnt1++;
	  while(key!=0x07);
		
		read_calib();
	
	//  return;
		
	/////////////////////CVT2_V//////////////////////////
		
		cmd_lcd(0x80);
	  display_lcd("CVT2_VC:");
	
    read_calib();	 //cnt++;
							
	  temp1=ratio_lmt_cvt2;
	
disp_chk_cvt2: 

    calib_lmt=temp1;

    CVT2_V=(unsigned long int)((CVT2_CNT*230)/calib_lmt); 

  //cmd_lcd(0x88);
	//lcd_integer(INV1_V);

	cmd_lcd(0XC0);
	lcd_integer(CVT2_V);
	write_lcd(' ');
	write_lcd(' ');
	
   if(key==0X05)  //if DOWN key press
  	 goto dec_shnt_cvt2;
	 else if(key==0X06)  //if UP key press
     goto inc_shnt_cvt2; //if SET key press
	 else if((key==0X03)&&(s_set1_flag==0))       
     goto s_set_cvt2;
	 else 
	   {goto disp_chk_cvt2;}

inc_shnt_cvt2:
  temp1+=0.05;	
	//cnt2++;  
   if(temp1>3000)//1450)
   temp1=3000;//1450;
	 s_set1_flag=0;
	 goto disp_chk_cvt2;

dec_shnt_cvt2:
  temp1-=0.05;	
	//cnt3++;
	 if(temp1<700)
   temp1=700;   
   s_set1_flag=0;  
   goto disp_chk_cvt2;

 s_set_cvt2:
    temp2=temp1;
    c1=temp2;
	  data_write(RATIO_LMT_CVT2L,c1);
	  delay1(20);
	  c1=temp2>>8;
	  data_write(RATIO_LMT_CVT2H,c1);
	  delay1(20);
	  s_set1_flag=1;  //cnt1++;
	  while(key!=0x07);
		
		read_calib();
	
	 // return;
		
		
	//////////////////////txr1/////////////////////

    cmd_lcd(0x80);
	  display_lcd("TXR1_VC:");
	
    read_calib();	 //cnt++;
							
	  temp1=ratio_lmt_t1;
	
disp_chk_txr1: 

    calib_lmt=temp1;

    TXR1_V=(unsigned long int)((TXR1_CNT*110)/calib_lmt); 

  
	cmd_lcd(0XC0);
	
	lcd_integer(TXR1_V);
	write_lcd(' ');
	
   if(key==0X05)  //if DOWN key press
  	 goto dec_shnt_txr1;
	 else if(key==0X06)  //if UP key press
     goto inc_shnt_txr1; //if SET key press
	 else if((key==0X03)&&(s_set1_flag==0))       
     goto s_set_txr1;
	 else 
	   {goto disp_chk_txr1;}

inc_shnt_txr1:
  temp1+=0.05;	
	//cnt2++;  
   if(temp1>3000)//1450)
   temp1=3000;//1450;
	 s_set1_flag=0;
	 goto disp_chk_txr1;

dec_shnt_txr1:
  temp1-=0.05;	
	//cnt3++;
	 if(temp1<700)
   temp1=700;   
   s_set1_flag=0;  
   goto disp_chk_txr1;

 s_set_txr1:
    temp2=temp1;
    c1=temp2;
	  data_write(RATIO_LMT_TXR1L,c1);
	  delay1(20);
	  c1=temp2>>8;
	  data_write(RATIO_LMT_TXR1H,c1);
	  delay1(20);
	  s_set1_flag=1;  //cnt1++;
	  while(key!=0x07);
		
		read_calib();
		
		
		
		//////////////////////txr2/////////////////////

  
		cmd_lcd(0x80);
	  display_lcd("TXR2_VC:");
	
    read_calib();	//cnt++;
							
	  temp1=ratio_lmt_txr2;
	
disp_chk_txr2: 

    calib_lmt=temp1;

    TXR2_V=(unsigned long int)((TXR2_CNT*110)/calib_lmt); 

  //cmd_lcd(0x88);
	//lcd_integer(INV1_V);

	cmd_lcd(0XC0);
	lcd_integer(TXR2_V);
	write_lcd(' ');
	write_lcd(' ');
	
   if(key==0X05)  //if DOWN key press
  	 goto dec_shnt_txr2;
	 else if(key==0X06)  //if UP key press
     goto inc_shnt_txr2; //if SET key press
	 else if((key==0X03)&&(s_set1_flag==0))       
     goto s_set_txr2;
	 else 
	   {goto disp_chk_txr2;}

inc_shnt_txr2:
  temp1+=0.05;	
	//cnt2++;  
   if(temp1>3000)//1450)
   temp1=3000;//1450;
	 s_set1_flag=0;
	 goto disp_chk_txr2;

dec_shnt_txr2:
  temp1-=0.05;	
	//cnt3++;
	 if(temp1<700)
   temp1=700;   
   s_set1_flag=0;  
   goto disp_chk_txr2;

 s_set_txr2:
    temp2=temp1;
    c1=temp2;
	  data_write(RATIO_LMT_TXR2L,c1);
	  delay1(20);
	  c1=temp2>>8;
	  data_write(RATIO_LMT_TXR2H,c1);
	  delay1(20);
	  s_set1_flag=1;  //cnt1++;
	  while(key!=0x07);
		
		read_calib();
		
		///////////////////////txr3/////////////////
		
		cmd_lcd(0x80);
	  display_lcd("TXR3_VC:");
	
    read_calib();	//cnt++;
							
	  temp1=ratio_lmt_t3;
	
disp_chk_txr3: 

    calib_lmt=temp1;

    TXR3_V=(unsigned long int)((TXR3_CNT*110)/calib_lmt); 

  //cmd_lcd(0x88);
	//lcd_integer(INV1_V);

	cmd_lcd(0XC0);
	lcd_integer(TXR3_V);
	write_lcd(' ');
	write_lcd(' ');
	
   if(key==0X05)  //if DOWN key press
  	 goto dec_shnt_txr3;
	 else if(key==0X06)  //if UP key press
     goto inc_shnt_txr3; //if SET key press
	 else if((key==0X03)&&(s_set1_flag==0))       
     goto s_set_txr3;
	 else 
	   {goto disp_chk_txr3;}

inc_shnt_txr3:
  temp1+=0.05;	
	//cnt2++;  
   if(temp1>3000)//1450)
   temp1=3000;//1450;
	 s_set1_flag=0;
	 goto disp_chk_txr3;

dec_shnt_txr3:
  temp1-=0.05;	
	//cnt3++;
	 if(temp1<700)
   temp1=700;   
   s_set1_flag=0;  
   goto disp_chk_txr3;

 s_set_txr3:
    temp2=temp1;
    c1=temp2;
	  data_write(RATIO_LMT_TXR3L,c1);
	  delay1(20);
	  c1=temp2>>8;
	  data_write(RATIO_LMT_TXR3H,c1);
	  delay1(20);
	  s_set1_flag=1;  //cnt1++;
	  while(key!=0x07);
		
		read_calib();
		
		///////////////////////txr4////////////////////////////////
		
		cmd_lcd(0x80);
	  display_lcd("TXR4_VC:");
	
    read_calib();	//cnt++;
							
	  temp1=ratio_lmt_t4;
	
disp_chk_txr4: 

    calib_lmt=temp1;

    TXR4_V=(unsigned long int)((TXR4_CNT*110)/calib_lmt); 

  //cmd_lcd(0x88);
	//lcd_integer(INV1_V);

	cmd_lcd(0XC0);
	lcd_integer(TXR4_V);
	write_lcd(' ');
	write_lcd(' ');
	
   if(key==0X05)  //if DOWN key press
  	 goto dec_shnt_txr4;
	 else if(key==0X06)  //if UP key press
     goto inc_shnt_txr4; //if SET key press
	 else if((key==0X03)&&(s_set1_flag==0))       
     goto s_set_txr4;
	 else 
	   {goto disp_chk_txr4;}

inc_shnt_txr4:
  temp1+=0.05;	
	//cnt2++;  
   if(temp1>3000)//1450)
   temp1=3000;//1450;
	 s_set1_flag=0;
	 goto disp_chk_txr4;

dec_shnt_txr4:
  temp1-=0.05;	
	//cnt3++;
	 if(temp1<700)
   temp1=700;   
   s_set1_flag=0;  
   goto disp_chk_txr4;

 s_set_txr4:
    temp2=temp1;
    c1=temp2;
	  data_write(RATIO_LMT_TXR4L,c1);
	  delay1(20);
	  c1=temp2>>8;
	  data_write(RATIO_LMT_TXR4H,c1);
	  delay1(20);
	  s_set1_flag=1;  //cnt1++;
	  while(key!=0x07);
		
		read_calib();
		
		///////////////////////txr5////////////////////////
		
		cmd_lcd(0x80);
	  display_lcd("TXR5_VC:");
	
    read_calib();	//cnt++;
							
	  temp1=ratio_lmt_t5;
	
disp_chk_txr5: 

    calib_lmt=temp1;

    TXR5_V=(unsigned long int)((TXR5_CNT*110)/calib_lmt); 

  //cmd_lcd(0x88);
	//lcd_integer(INV1_V);

	cmd_lcd(0XC0);
	lcd_integer(TXR5_V);
	write_lcd(' ');
	write_lcd(' ');
	
   if(key==0X05)  //if DOWN key press
  	 goto dec_shnt_txr5;
	 else if(key==0X06)  //if UP key press
     goto inc_shnt_txr5; //if SET key press
	 else if((key==0X03)&&(s_set1_flag==0))       
     goto s_set_txr5;
	 else 
	   {goto disp_chk_txr5;}

inc_shnt_txr5:
  temp1+=0.05;	
	//cnt2++;  
   if(temp1>3000)//1450)
   temp1=3000;//1450;
	 s_set1_flag=0;
	 goto disp_chk_txr5;

dec_shnt_txr5:
  temp1-=0.05;	
	//cnt3++;
	 if(temp1<700)
   temp1=700;   
   s_set1_flag=0;  
   goto disp_chk_txr5;

 s_set_txr5:
    temp2=temp1;
    c1=temp2;
	  data_write(RATIO_LMT_TXR5L,c1);
	  delay1(20);
	  c1=temp2>>8;
	  data_write(RATIO_LMT_TXR5H,c1);
	  delay1(20);
	  s_set1_flag=1;  //cnt1++;
	  while(key!=0x07);
		
		read_calib();
		
		//////////////////////////txr6////////////////////////////
		
		cmd_lcd(0x80);
	  display_lcd("TXR6_VC:");
	
    read_calib();	//cnt++;
							
	  temp1=ratio_lmt_t6;
	
disp_chk_txr6: 

    calib_lmt=temp1;

    TXR6_V=(unsigned long int)((TXR6_CNT*110)/calib_lmt); 

  //cmd_lcd(0x88);
	//lcd_integer(INV1_V);

	cmd_lcd(0XC0);
	lcd_integer(TXR6_V);
	write_lcd(' ');
	write_lcd(' ');
	
   if(key==0X05)  //if DOWN key press
  	 goto dec_shnt_txr6;
	 else if(key==0X06)  //if UP key press
     goto inc_shnt_txr6; //if SET key press
	 else if((key==0X03)&&(s_set1_flag==0))       
     goto s_set_txr6;
	 else 
	   {goto disp_chk_txr6;}

inc_shnt_txr6:
  temp1+=0.05;	
	//cnt2++;  
   if(temp1>3000)//1450)
   temp1=3000;//1450;
	 s_set1_flag=0;
	 goto disp_chk_txr6;

dec_shnt_txr6:
  temp1-=0.05;	
	//cnt3++;
	 if(temp1<700)
   temp1=700;   
   s_set1_flag=0;  
   goto disp_chk_txr6;

 s_set_txr6:
    temp2=temp1;
    c1=temp2;
	  data_write(RATIO_LMT_TXR6L,c1);
	  delay1(20);
	  c1=temp2>>8;
	  data_write(RATIO_LMT_TXR6H,c1);
	  delay1(20);
	  s_set1_flag=1;  //cnt1++;
	  while(key!=0x07);
		
		read_calib();
		
		/////////////////txr7////////////////////
		
		cmd_lcd(0x80);
	  display_lcd("TXR7_VC:");
	
    read_calib();	//cnt++;
							
	  temp1=ratio_lmt_t7;
	
disp_chk_txr7: 

    calib_lmt=temp1;

    TXR7_V=(unsigned long int)((TXR7_CNT*110)/calib_lmt); 

  //cmd_lcd(0x88);
	//lcd_integer(INV1_V);

	cmd_lcd(0XC0);
	lcd_integer(TXR7_V);
	write_lcd(' ');
	write_lcd(' ');
	
   if(key==0X05)  //if DOWN key press
  	 goto dec_shnt_txr7;
	 else if(key==0X06)  //if UP key press
     goto inc_shnt_txr7; //if SET key press
	 else if((key==0X03)&&(s_set1_flag==0))       
     goto s_set_txr7;
	 else 
	   {goto disp_chk_txr7;}

inc_shnt_txr7:
  temp1+=0.05;	
	//cnt2++;  
   if(temp1>3000)//1450)
   temp1=3000;//1450;
	 s_set1_flag=0;
	 goto disp_chk_txr7;

dec_shnt_txr7:
  temp1-=0.05;	
	//cnt3++;
	 if(temp1<700)
   temp1=700;   
   s_set1_flag=0;  
   goto disp_chk_txr7;

 s_set_txr7:
    temp2=temp1;
    c1=temp2;
	  data_write(RATIO_LMT_TXR7L,c1);
	  delay1(20);
	  c1=temp2>>8;
	  data_write(RATIO_LMT_TXR7H,c1);
	  delay1(20);
	  s_set1_flag=1;  //cnt1++;
	  while(key!=0x07);
		
		read_calib();

///////////////////////////txr8////////////////////////

    cmd_lcd(0x80);
	  display_lcd("TXR8_VC:");
	
    read_calib();	//cnt++;
							
	  temp1=ratio_lmt_t8;
	
disp_chk_txr8: 

    calib_lmt=temp1;

    TXR8_V=(unsigned long int)((TXR8_CNT*110)/calib_lmt); 

  //cmd_lcd(0x88);
	//lcd_integer(INV1_V);

	cmd_lcd(0XC0);
	lcd_integer(TXR8_V);
	write_lcd(' ');
	write_lcd(' ');
	
   if(key==0X05)  //if DOWN key press
  	 goto dec_shnt_txr8;
	 else if(key==0X06)  //if UP key press
     goto inc_shnt_txr8; //if SET key press
	 else if((key==0X03)&&(s_set1_flag==0))       
     goto s_set_txr8;
	 else 
	   {goto disp_chk_txr8;}

inc_shnt_txr8:
  temp1+=0.05;	
	//cnt2++;  
   if(temp1>3000)//1450)
   temp1=3000;//1450;
	 s_set1_flag=0;
	 goto disp_chk_txr8;

dec_shnt_txr8:
  temp1-=0.05;	
	//cnt3++;
	 if(temp1<700)
   temp1=700;   
   s_set1_flag=0;  
   goto disp_chk_txr8;

 s_set_txr8:
    temp2=temp1;
    c1=temp2;
	  data_write(RATIO_LMT_TXR8L,c1);
	  delay1(20);
	  c1=temp2>>8;
	  data_write(RATIO_LMT_TXR8H,c1);
	  delay1(20);
	  s_set1_flag=1;  //cnt1++;
	  while(key!=0x07);
		
		read_calib();


	  return;		
		
} 



void delay(unsigned int D)
{
	unsigned int i;
	while(D)
	{
		for(i=0;i<16;i++);
		D--;
	}
	
 return;	
}


void timer_sec(void)
{ 
   TIM4_IER=0X01;
	 TIM4_EGR=0X01;
	 TIM4_ARR=250;      // Every 1ms Timer4 overflow Interrupt
	 TIM4_PSCR=0x06;	  //
	 TIM4_CNTR=0X00;   
	 TIM4_CR1=0X81;  //auto reload,counter enable
}

@far @interrupt void usart_tx(void)
{	
 	 if((txdata & 0X100)==0X100)    
   {  
     c_tx_counter1=1;	 
		 flag12=1;	
	 } 
	 
  	  txdata=0;
		  txdata=sdata[c_tx_counter1];
		  USART1_DR=txdata;
		 
		  c_tx_counter1++;
		  if(c_tx_counter1>51)
		  {c_tx_counter1=0; TX_enable=0;}	 //TX_enable=0;

  delay(3);
		
	USART1_SR=0;
	 
	return;
}

@far @interrupt void usart_rx(void)
{
 /*
  
	  if(RXNE==1)
	  {
	    data1=USART1_DR;
	 
	 
      rdata[c_rx_counter1]=data1; 
      rdata1[c_rx_counter1]=data1; 			
      ++c_rx_counter1;
			 
      if(c_rx_counter1>8)    //8              
      {cou1=50;c_rx_counter1=0;} 
  

		  data11=rdata[3];
	  }*/	
		
		
	 if((PE==0)||(FE==0)||(NF==0)||(OR==0)&&(RXNE==1)&&(c_rx_counter1==0))	
   {
	   computer=0X00;
   }
	  data1=USART1_DR;
	 
	  if(computer==0X00)                                         
    {
      rdata[c_rx_counter1-1]=data1;
		//	rdata1[c_rx_counter1-1]=data1;
			
      ++c_rx_counter1;       
			 
      if(c_rx_counter1>9)       //NO_BYETES_HDP           
      {cou1=80;}       
    }     
	 
	//  data11=rdata[3];
	USART1_SR=0;
	            
	 
	return;
}


@far @interrupt void Timer4_ovf(void)      //Timer4_ovf as LABLE
{
   static unsigned char cnt_disp=0; 
	
    TIM4_SR1=0X00;  
		
		DISP++;
		cnt_KEY++;
		
	  counter++;
		if(counter>=77)
		counter=1;
		
		if(cou1!=0)
    {
			cou1--;
			if(cou1<76 && cou1>=74)
			{RS485_ENABLE=0; RX_enable=0;} 
			if(cou1<74)  // && cou1>1)
			{flag10=1;}
		
			if(cou1==0)
			{RX_enable=1; RS485_ENABLE=0; }
		 
			if(cou1==0) 
			 c_rx_counter1=0;
    }
		
		if(flg_set1)
		{
       cntr0--;
			 if(cntr0==0x00)
	     {
	        flg_set1=0;
		   }
	  }	

    if(cnt_KEY<30)    
    {
      ; 
    } 	 
	  else if (cnt_KEY>=30)
    { 
			 unsigned char c;
	
			 c = key_deb2;
			 key_deb2 =(unsigned char)(PD_IDR&0x07);	  
						
			 if(key_deb2==c)
			 {
					c=key_deb1;
					key_deb1=key_deb2;	  
					
					if(key_deb1==c)
					{
						c=key_deb0;
						key_deb0=key_deb1;
						if(key_deb0==c)
						key=key_deb0;
					}
			 }
			 cnt_KEY=0;		 
    }
		    
				
        
		
    	      switch(flag5)
						{
							case 0:         
										if(V0==0)
										{								        										

											  dc_cnt+=adc_values[0];   //channel 0 600; 
												dc_cntr++;   
												
												if(dc_cntr>=200)
												{
													dc_cnt1=(unsigned long int)(dc_cnt/200);													
													
													TXR4_CNT=(unsigned long int)dc_cnt1;
													
												  if(TXR4_CNT<780)
													TXR4_V=(unsigned int)((dc_cnt1*100)/780);
											
													else													
													TXR4_V=(unsigned int)((dc_cnt1*110)/ratio_lmt_t44);
													
													if(TXR4_V<100)
													TXR4_V=TXR4_V+4;	
													
													if(TXR4_V>=130)
													TXR4_V=TXR4_V-6;	
													
													if(TXR4_V>=120)
													TXR4_V=TXR4_V-4;

                         	if(TXR4_V<=60)
													TXR4_V=0;
													
													Comm_Txr4_vlt=TXR4_V;
													
													
													dc_cnt=0;dc_cnt1=0;
													dc_cntr=0; 
													flag5=1; MUL_A0=1; MUL_A1=0; MUL_A2=0; MUL_A3=0;   //Y1
												}
										}		
							      
										break;
							case 1:     
									  if(V1==0)
										{
												dc_cnt+=adc_values[0];   //channel 0 600; 
												dc_cntr++;   
												
												if(dc_cntr>=200)
												{
													dc_cnt1=(unsigned long int)(dc_cnt/200);
													TXR3_CNT=(unsigned long int)dc_cnt1;
													
													if(TXR3_CNT<780)
													TXR3_V=(unsigned int)((dc_cnt1*100)/780);
											
													else													
													TXR3_V=(unsigned int)((dc_cnt1*110)/ratio_lmt_t33);
													
													if(TXR3_V<100)
													TXR3_V=TXR3_V+4;	
													
													if(TXR3_V>=130)
													TXR3_V=TXR3_V-6;	
													
													if(TXR3_V>=120)
													TXR3_V=TXR3_V-4;
													
													if(TXR3_V<=60)
													TXR3_V=0;

                          Comm_Txr3_vlt=TXR3_V;
												
												
													dc_cnt=0;dc_cnt1=0;
													dc_cntr=0; 
													flag5=2; MUL_A0=0; MUL_A1=1; MUL_A2=0; MUL_A3=0;   //Y2
												}
									  }	
                    break;										
							case 2:
										if(V2==0)
										{
												dc_cnt+=adc_values[0];   //channel 0 600; 
												dc_cntr++;   
												
												if(dc_cntr>=200)
												{
													dc_cnt1=(unsigned long int)(dc_cnt/200);
													TXR2_CNT=(unsigned long int)dc_cnt1;
													
													if(TXR2_CNT<780)
													TXR2_V=(unsigned int)((dc_cnt1*100)/780);
										
													else 
													TXR2_V=(unsigned int)((dc_cnt1*110)/ratio_lmt_txr22);
													
													if(TXR2_V<100)
													TXR2_V=TXR2_V+4;

                          if(TXR2_V>=130)
													TXR2_V=TXR2_V-6;
													
													if(TXR2_V>=120)
													TXR2_V=TXR2_V-4;
													
													if(TXR2_V<=60)
													TXR2_V=0;

                          Comm_Txr2_vlt=TXR2_V;
													
											
													dc_cnt=0;dc_cnt1=0;
													dc_cntr=0; 
													flag5=3; MUL_A0=1; MUL_A1=1; MUL_A2=0; MUL_A3=0;   //Y3
												}	
									  }	
                    break;										
							case 3:
										if(V3==0)
										{
												dc_cnt+=adc_values[0];   //channel 0 600; 
												dc_cntr++;   
												
												if(dc_cntr>=200)
												{
													dc_cnt1=(unsigned long int)(dc_cnt/200);
													TXR1_CNT=(unsigned long int)dc_cnt1;
													
													if(TXR1_CNT<780)
													TXR1_V=(unsigned int)((dc_cnt1*100)/780);
										
													else 													
													TXR1_V=(unsigned int)((dc_cnt1*110)/ratio_lmt_t11);
													
													if(TXR1_V<100)
													TXR1_V=TXR1_V+4;	
													
													if(TXR1_V>=130)
													TXR1_V=TXR1_V-6;	
													
													if(TXR1_V>=120)
													TXR1_V=TXR1_V-4;	
													
													if(TXR1_V<=60)
													TXR1_V=0;
													
													Comm_Txr1_vlt=TXR1_V;
													
												
													dc_cnt=0;dc_cnt1=0;
													dc_cntr=0; 
													flag5=4; MUL_A0=0; MUL_A1=0; MUL_A2=1; MUL_A3=0;   //Y4
												}
									  }	
                    break;										
							case 4:
										if(V4==0)
										{
												dc_cnt+=adc_values[0];   //channel 0 600; 
												dc_cntr++;   
												
												if(dc_cntr>=200)
												{
													dc_cnt1=(unsigned long int)(dc_cnt/200);
													INV2_CNT=(unsigned long int)dc_cnt1;
													
													if(INV2_CNT<1764)
													INV2_V=(unsigned int)((dc_cnt1*180)/1764);													
													else if(INV2_CNT<1887)
													INV2_V=(unsigned int)((dc_cnt1*190)/1887);
													else if(INV2_CNT<2007)
													INV2_V=(unsigned int)((dc_cnt1*200)/2007);
													else if(INV2_CNT<2120)
													INV2_V=(unsigned int)((dc_cnt1*210)/2120);
													else if(INV2_CNT<2246)
													INV2_V=(unsigned int)((dc_cnt1*220)/2246);													
													else   //if(INV2_CNT<2353)
													INV2_V=(unsigned int)((dc_cnt1*230)/ratio_lmt_inv2);
													
										
													if(INV2_V<=210)
													INV2_V=INV2_V+2;		

                          if(INV2_V<=60)
													INV2_V=0;	
                          
                          Comm_Inv2_vlt=INV2_V;													
												
													dc_cnt=0;dc_cnt1=0;  //s=0;
													dc_cntr=0; 
													flag5=5; MUL_A0=1; MUL_A1=0; MUL_A2=1; MUL_A3=0;   //Y5
												}
									  }	
                    break;					
							case 5:
										if(V5==0)
										{
												dc_cnt+=adc_values[0];   //channel 0 600; 
												dc_cntr++;   
												
												if(dc_cntr>=200)
												{
													dc_cnt1=(unsigned long int)(dc_cnt/200);
													INV1_CNT=(unsigned long int)dc_cnt1;
													
													if(INV1_CNT<1764)
													INV1_V=(unsigned int)((dc_cnt1*180)/1764);													
													else if(INV1_CNT<1887)
													INV1_V=(unsigned int)((dc_cnt1*190)/1887);
													else if(INV1_CNT<2007)
													INV1_V=(unsigned int)((dc_cnt1*200)/2007);
													else if(INV1_CNT<2120)
													INV1_V=(unsigned int)((dc_cnt1*210)/2120);
													else if(INV1_CNT<2246)
													INV1_V=(unsigned int)((dc_cnt1*220)/2246);
													else   //if(INV1_CNT<2353)
													INV1_V=(unsigned int)((dc_cnt1*230)/ratio_lmt_inv1);
													
										
													if(INV1_V<=210)
													INV1_V=INV1_V+2;
													
													if(INV1_V<=60)
													INV1_V=0;
													
													Comm_Inv1_vlt=INV1_V;
											
													dc_cnt=0;dc_cnt1=0; //s=0;
													dc_cntr=0; 
													flag5=6; MUL_A0=0; MUL_A1=1; MUL_A2=1; MUL_A3=0;   //Y6
													
												}
									  }	
                    break;					
							case 6:
										if(V6==0)
										{
												dc_cnt+=adc_values[0];   //channel 0 600; 
												dc_cntr++;   
												
												if(dc_cntr>=200)
												{
													dc_cnt1=(unsigned long int)(dc_cnt/200);
													CVT2_CNT=(unsigned long int)dc_cnt1;
													CVT2_V=(unsigned int)((dc_cnt1*230)/ratio_lmt_cvt22);
													
													if(CVT2_V<=60)
													CVT2_V=0;
													
													Comm_Cvt2_vlt=CVT2_V;
													
													dc_cnt=0;dc_cnt1=0; //s=0;
													dc_cntr=0; 
													flag5=7; MUL_A0=1; MUL_A1=1; MUL_A2=1; MUL_A3=0;   //Y7
												}
									  }	
                    break;					
							case 7:
										if(V7==0)
										{
												dc_cnt+=adc_values[0];   //channel 0 600; 
												dc_cntr++;   
												
												if(dc_cntr>=200)
												{
													dc_cnt1=(unsigned long int)(dc_cnt/200);
													CVT1_CNT=(unsigned long int)dc_cnt1;
													
													CVT1_V=(unsigned int)((dc_cnt1*230)/ratio_lmt2_cvt1);
											
													if(CVT1_V<=210)
													CVT1_V=CVT1_V+2;
													
													if(CVT1_V<=60)
													CVT1_V=0;

                          Comm_Cvt1_vlt=CVT1_V;
										
													dc_cnt=0;dc_cnt1=0; //s=0;
													dc_cntr=0; 
													flag5=12; MUL_A0=0; MUL_A1=0; MUL_A2=1; MUL_A3=1;   //Y12
												}
									  }	
                    break;					
							case 12:
										if(V8==0)
										{
											  dc_cnt+=adc_values[0];   //channel 0 600; 
												dc_cntr++;   
												
												if(dc_cntr>=200)
												{
													dc_cnt1=(unsigned long int)(dc_cnt/200);
													TXR8_CNT=(unsigned long int)dc_cnt1;
													
													
													if(TXR8_CNT<780)
													TXR8_V=(unsigned int)((dc_cnt1*100)/780);
										
													else													
													TXR8_V=(unsigned int)((dc_cnt1*110)/ratio_lmt_t88);
													
													if(TXR8_V<100)
													TXR8_V=TXR8_V+4;	
													
													if(TXR8_V>=130)
													TXR8_V=TXR8_V-6;	
													
													if(TXR8_V>=120)
													TXR8_V=TXR8_V-4;				

                         	if(TXR8_V<=60)
													TXR8_V=0;			 
													
													Comm_Txr8_vlt=TXR8_V;
													
													dc_cnt=0;dc_cnt1=0; //s=0;
													dc_cntr=0; 
													flag5=13; MUL_A0=1; MUL_A1=0; MUL_A2=1; MUL_A3=1;   //Y13
												}
									  }	
                    break;					
							case 13:
										if(V13==0)
										{
												dc_cnt+=adc_values[0];   //channel 0 600; 
												dc_cntr++;   
												
												if(dc_cntr>=200)
												{
													dc_cnt1=(unsigned long int)(dc_cnt/200);
													TXR7_CNT=(unsigned long int)dc_cnt1;
													
													if(TXR7_CNT<780)
													TXR7_V=(unsigned int)((dc_cnt1*100)/780);
											
													else													
													TXR7_V=(unsigned int)((dc_cnt1*110)/ratio_lmt_t77);
													
													if(TXR7_V<100)
													TXR7_V=TXR7_V+4;	
													
													if(TXR7_V>=130)
													TXR7_V=TXR7_V-6;	
													
													if(TXR7_V>=120)
													TXR7_V=TXR7_V-4;
													
													if(TXR7_V<=60)
													TXR7_V=0;

                          Comm_Txr7_vlt=TXR7_V;
													
													dc_cnt=0;dc_cnt1=0;
													dc_cntr=0; 
													flag5=14; MUL_A0=0; MUL_A1=1; MUL_A2=1; MUL_A3=1;   //Y14
												}
									  }	
                    break;					
							case 14:
							      if(V14==0)
										{
												dc_cnt+=adc_values[0];   //channel 0 600; 
												dc_cntr++;   
												
												if(dc_cntr>=200)
												{
													dc_cnt1=(unsigned long int)(dc_cnt/200);
													TXR6_CNT=(unsigned long int)dc_cnt1;									
																										
													if(TXR6_CNT<780)
													TXR6_V=(unsigned int)((dc_cnt1*100)/780);
											
													else													
													TXR6_V=(unsigned int)((dc_cnt1*110)/ratio_lmt_t66);
													
													if(TXR6_V<100)
													TXR6_V=TXR6_V+4;	
													
													if(TXR6_V>=130)
													TXR6_V=TXR6_V-6;	
													
													if(TXR6_V>=120)
													TXR6_V=TXR6_V-4;	
													
													if(TXR6_V<=60)
													TXR6_V=0;	
													
													Comm_Txr6_vlt=TXR6_V;
													
													dc_cnt=0;dc_cnt1=0; //s=0;
													dc_cntr=0; 
													flag5=15; MUL_A0=1; MUL_A1=1; MUL_A2=1; MUL_A3=1;   //Y15
												}
									  }	
                    break;					
							case 15:
										if(V15==0)
										{
												dc_cnt+=adc_values[0];   //channel 0 600; 
												dc_cntr++;   
												
												if(dc_cntr>=200)
												{
													dc_cnt1=(unsigned long int)(dc_cnt/200);
													TXR5_CNT=(unsigned long int)dc_cnt1;
													
													
													if(TXR5_CNT<780)
													TXR5_V=(unsigned int)((dc_cnt1*100)/780);
											
													else													
													TXR5_V=(unsigned int)((dc_cnt1*110)/ratio_lmt_t55);
													
													if(TXR5_V<100)
													TXR5_V=TXR5_V+4;	
													
													if(TXR5_V>=130)
													TXR5_V=TXR5_V-6;	
													
													if(TXR5_V>=120)
													TXR5_V=TXR5_V-4;
													
													if(TXR5_V<=60)
													TXR5_V=0;

                          Comm_Txr5_vlt=TXR5_V;
											
													dc_cnt=0;dc_cnt1=0; ///s=0;
													dc_cntr=0; 
													flag5=0; MUL_A0=0; MUL_A1=0; MUL_A2=0; MUL_A3=0;   //Y0
												}   
									  }	
                 
              
                   							  
						}
						
					m_sec_disp++;					
		
					//////////////////////cvt1/////////////////////////
					
					Cvt1_sample_cnt=adc_values[1]; 
					if(Cvt1_sample_cnt<2048)
			    Cvt1_sample_cnt1=(unsigned long int)(2048-Cvt1_sample_cnt);
				  else if(Cvt1_sample_cnt>=2048)
				  Cvt1_sample_cnt1=(unsigned long int)(Cvt1_sample_cnt-2048);				
					Cvt1_square_cnt+=(Cvt1_sample_cnt1*Cvt1_sample_cnt1);
					
					/////////////cvt2///////////////////////////////////
					
					Cvt2_sample_cnt=adc_values[2]; 
					if(Cvt2_sample_cnt<2048)
			    Cvt2_sample_cnt1=(unsigned long int)(2048-Cvt2_sample_cnt);
				  else if(Cvt2_sample_cnt>=2048)
				  Cvt2_sample_cnt1=(unsigned long int)(Cvt2_sample_cnt-2048);				
					Cvt2_square_cnt+=(Cvt2_sample_cnt1*Cvt2_sample_cnt1);
					
					////////////////
					
				  
				  Inv1_sample_cnt=adc_values[12]; 
					if(Inv1_sample_cnt<2048)
			    Inv1_sample_cnt1=(unsigned long int)(2048-Inv1_sample_cnt);
				  else if(Inv1_sample_cnt>=2048)
				  Inv1_sample_cnt1=(unsigned long int)(Inv1_sample_cnt-2048);				
					Inv1_square_cnt+=(Inv1_sample_cnt1*Inv1_sample_cnt1);
					
					
					//////////////////////////////////inv2///////////////////
				
					Inv2_sample_cnt=adc_values[3]; 
					if(Inv2_sample_cnt<2048)
			    Inv2_sample_cnt1=(unsigned long int)(2048-Inv2_sample_cnt);
				  else if(Inv2_sample_cnt>=2048)
				  Inv2_sample_cnt1=(unsigned long int)(Inv2_sample_cnt-2048);				
					Inv2_square_cnt+=(Inv2_sample_cnt1*Inv2_sample_cnt1);
					
					
					////////////////txr1//////////////////			
					
					
					Txr1_sample_cnt=adc_values[4]; 
					if(Txr1_sample_cnt<2048)
			    Txr1_sample_cnt1=(unsigned long int)(2048-Txr1_sample_cnt);
				  else if(Txr1_sample_cnt>=2048)
				  Txr1_sample_cnt1=(unsigned long int)(Txr1_sample_cnt-2048);				
					Txr1_square_cnt+=(Txr1_sample_cnt1*Txr1_sample_cnt1);
					
					//////////////////txr2//////////////////////
					
					Txr2_sample_cnt=adc_values[5]; 
					if(Txr2_sample_cnt<2048)
			    Txr2_sample_cnt1=(unsigned long int)(2048-Txr2_sample_cnt);
				  else if(Txr2_sample_cnt>=2048)
				  Txr2_sample_cnt1=(unsigned long int)(Txr2_sample_cnt-2048);				
					Txr2_square_cnt+=(Txr2_sample_cnt1*Txr2_sample_cnt1);

					//////////////////txr3//////////////////////
					
					Txr3_sample_cnt=adc_values[6]; 
					if(Txr3_sample_cnt<2048)
			    Txr3_sample_cnt1=(unsigned long int)(2048-Txr3_sample_cnt);
				  else if(Txr3_sample_cnt>=2048)
				  Txr3_sample_cnt1=(unsigned long int)(Txr3_sample_cnt-2048);				
					Txr3_square_cnt+=(Txr3_sample_cnt1*Txr3_sample_cnt1);

					//////////////////txr4//////////////////////
					
					Txr4_sample_cnt=adc_values[7]; 
					if(Txr4_sample_cnt<2048)
			    Txr4_sample_cnt1=(unsigned long int)(2048-Txr4_sample_cnt);
				  else if(Txr4_sample_cnt>=2048)
				  Txr4_sample_cnt1=(unsigned long int)(Txr4_sample_cnt-2048);				
					Txr4_square_cnt+=(Txr4_sample_cnt1*Txr4_sample_cnt1);

          //////////////////txr5//////////////////////
					
					Txr5_sample_cnt=adc_values[8]; 
					if(Txr5_sample_cnt<2048)
			    Txr5_sample_cnt1=(unsigned long int)(2048-Txr5_sample_cnt);
				  else if(Txr5_sample_cnt>=2048)
				  Txr5_sample_cnt1=(unsigned long int)(Txr5_sample_cnt-2048);				
					Txr5_square_cnt+=(Txr5_sample_cnt1*Txr5_sample_cnt1);
					
					 //////////////////txr6//////////////////////
					
					Txr6_sample_cnt=adc_values[9]; 
					if(Txr6_sample_cnt<2048)
			    Txr6_sample_cnt1=(unsigned long int)(2048-Txr6_sample_cnt);
				  else if(Txr6_sample_cnt>=2048)
				  Txr6_sample_cnt1=(unsigned long int)(Txr6_sample_cnt-2048);				
					Txr6_square_cnt+=(Txr6_sample_cnt1*Txr6_sample_cnt1);
					
					 //////////////////txr7//////////////////////
					
					Txr7_sample_cnt=adc_values[10]; 
					if(Txr7_sample_cnt<2048)
			    Txr7_sample_cnt1=(unsigned long int)(2048-Txr7_sample_cnt);
				  else if(Txr7_sample_cnt>=2048)
				  Txr7_sample_cnt1=(unsigned long int)(Txr7_sample_cnt-2048);				
					Txr7_square_cnt+=(Txr7_sample_cnt1*Txr7_sample_cnt1);
					
					 //////////////////txr8//////////////////////
					
				  Txr8_sample_cnt=adc_values[11];
					if(Txr8_sample_cnt<2048)
			    Txr8_sample_cnt1=(unsigned long int)(2048-Txr8_sample_cnt);
				  else if(Txr8_sample_cnt>=2048)
				  Txr8_sample_cnt1=(unsigned long int)(Txr8_sample_cnt-2048);				
					Txr8_square_cnt+=(Txr8_sample_cnt1*Txr8_sample_cnt1);
				
			
		DMA1_C1CR&=0XFE;               
		DMA1_C1NDTR=13;
		DMA1_C1CR|=0x01;

		START_CONV=1;	//ADC convertion starts at every 1ms
		
	

		
		disp_cntr1++;
		
 return;

   
} 

void ADC_INIT(void)
{
   ADC1_CR1=0X01;  //continues conv mode and adon	 
	 ADC1_CR2=0X07;  //384 adc clock cycles
	 ADC1_SQR4=0XF7;        //ch  0
	 ADC1_SQR3=0X87;        //ch  0
	 ADC1_SQR2=0X05;        //ch  0
	 
	 SYSCFG_RMPCR1=0X21;
}	 

void DMA_INIT(void) //For ADC 
{
	static unsigned int temp;
		
	 DMA1_C1CR=0x20; //memory incr, peripheral to memory
	 DMA1_C1SPR=0X08; //high priority and 16 bit mode 
	 DMA1_C1NDTR=13; 
		
	 temp=(unsigned int)&ADC1_DRH;
	 DMA1_C1PAR=(unsigned int)(temp&0xFFFF);
	 temp=(unsigned int)adc_values;
	 DMA1_C1M0AR=(unsigned int)(temp&0xFFFF);
}

void comm(void)
{
	USART1_BRR2=0X03;  //0X368=9600, 0X134=19200,1BA0
	USART1_BRR1=0X68;
	USART1_DR=0X00;

	RX_enable=1;
	RS485_ENABLE=0;
	
	USART1_CR2|=0X60;  //TX CMPLETE INTR ENABLE & RX 
}

void init_port(void)
{
	 PA_DDR=0X00;
   PA_CR1=0X0F;    
	 
   PB_DDR=0XF2;
   PB_CR1=0XF2;      
	 
	 PC_DDR=0XA0;
   PC_CR1=0XA0; 
	 
	 PE_DDR=0X3F;
   PE_CR1=0X3F;              
   PE_CR2=0X3F;              
        
	 PD_DDR=0X00;
   PD_CR1=0X00;   //FF          
   PD_CR2=0X00;             
}   

void CLOCK_CONFG(void)
{
	CLK_ICKCR=0X01;
	CLK_SWR=0X01;
	CLK_CKDIVR=0X00;
	CLK_SWCR=0X02;
	while(HSI_RDY==0);
	CLK_PCKENR1=0X24; //TIM4 clock enable
	CLK_PCKENR2=0X91; //DMA,ADC1 clock enable
}          
 
void disp_para(void)
{  

		  LCD_CLEAR();
		disp_cntr=15;
		  if(disp_cntr==0)
			{
			
				cmd_lcd(0x80);
				display_lcd("MComm");
				cmd_lcd(0xC0);
				lcd_integer(data11);
				write_lcd(' ');
				write_lcd(' ');
			}				
			
			else if(disp_cntr==1)
			{
			
				cmd_lcd(0x80);
				display_lcd("INV1_V:");
				cmd_lcd(0xC0);
				lcd_integer(INV1_V);
				write_lcd(' ');
				write_lcd(' ');
			}
			else if(disp_cntr==2)
			{
			
				cmd_lcd(0x80);
				display_lcd("INV2_V:");
				cmd_lcd(0xC0);
				lcd_integer(INV2_V);
				write_lcd(' ');
				write_lcd(' ');
				
			}
			else if(disp_cntr==3)
			{
			
				cmd_lcd(0x80);
				display_lcd("CVT1_V:");
				cmd_lcd(0xC0);
				lcd_integer(CVT1_V);
				write_lcd(' ');
				write_lcd(' ');
			
			}
			else if(disp_cntr==4)
			{
			
				cmd_lcd(0x80);
				display_lcd("CVT2_V:");
				cmd_lcd(0xC0);
				lcd_integer(CVT2_V);
				write_lcd(' ');
				write_lcd(' ');
				
			}
			else if(disp_cntr==5)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR1_V:");
				cmd_lcd(0xC0);
				lcd_integer(TXR1_V);
				write_lcd(' ');
				write_lcd(' ');
			
			}
			else if(disp_cntr==6)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR2_V:");
				cmd_lcd(0xC0);
				lcd_integer(TXR2_V);
				write_lcd(' ');
				write_lcd(' ');
			
			}
			else if(disp_cntr==7)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR3_V:");
				cmd_lcd(0xC0);
				lcd_integer(TXR3_V);
				write_lcd(' ');
				write_lcd(' ');
			
			}
			else if(disp_cntr==8)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR4_V:");
				cmd_lcd(0xC0);
				lcd_integer(TXR4_V);
				write_lcd(' ');
				write_lcd(' ');
				
			}
			else if(disp_cntr==9)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR5_V:");
				cmd_lcd(0xC0);
				lcd_integer(TXR5_V);
				write_lcd(' ');
				write_lcd(' ');
				
			}
			else if(disp_cntr==10)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR6_V:");
				cmd_lcd(0xC0);
				lcd_integer(TXR6_V);
				write_lcd(' ');
				write_lcd(' ');
				
			}
			else if(disp_cntr==11)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR7_V:");
				cmd_lcd(0xC0);
				lcd_integer(TXR7_V);
				write_lcd(' ');
				write_lcd(' ');
				
			}
			else if(disp_cntr==12)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR8_V:");
				cmd_lcd(0xC0);
				lcd_integer(TXR8_V);
				write_lcd(' ');
				write_lcd(' ');
			
			}
			else if(disp_cntr==13)
			{
			
				cmd_lcd(0x80);
				display_lcd("INV2_A:");
				cmd_lcd(0xC0);
				lcd_float(Inv2_current);
				write_lcd(' ');
				write_lcd(' ');
			}
			else if(disp_cntr==14)
			{
			
				cmd_lcd(0x80);
				display_lcd("CVT1_A:");
				cmd_lcd(0xC0);
				lcd_float(Cvt1_current);
				write_lcd(' ');
				write_lcd(' ');
			
			}
			else if(disp_cntr==15)
			{
			
				cmd_lcd(0x80);
				display_lcd("CVT2_A:");
				cmd_lcd(0xC0);
				lcd_float(Cvt2_current);
				write_lcd(' ');
				write_lcd(' ');
				
			}
			else if(disp_cntr==16)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR1_A:");
				cmd_lcd(0xC0);
				lcd_float(Txr1_current);
				write_lcd(' ');
				write_lcd(' ');
			
			}
			else if(disp_cntr==17)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR2_A:");
				cmd_lcd(0xC0);
				lcd_float(Txr2_current);
				write_lcd(' ');
				write_lcd(' ');
			
			}
			else if(disp_cntr==18)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR3_A:");
				cmd_lcd(0xC0);
				lcd_float(Txr3_current);
				write_lcd(' ');
				write_lcd(' ');
			
			}
			else if(disp_cntr==19)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR4_A:");
				cmd_lcd(0xC0);
				lcd_float(Txr4_current);
				write_lcd(' ');
				write_lcd(' ');
				
			}
			else if(disp_cntr==20)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR5_A:");
				cmd_lcd(0xC0);
				lcd_float(Txr5_current);
				write_lcd(' ');
				write_lcd(' ');
				
			}
			else if(disp_cntr==21)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR6_A:");
				cmd_lcd(0xC0);
				lcd_float(Txr6_current);
				write_lcd(' ');
				write_lcd(' ');
				
			}
			else if(disp_cntr==22)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR7_A:");
				cmd_lcd(0xC0);
				lcd_float(Txr7_current);
				write_lcd(' ');
				write_lcd(' ');
				
			}
			else if(disp_cntr==23)
			{
			
				cmd_lcd(0x80);
				display_lcd("TXR8_A:");
				cmd_lcd(0xC0);
				lcd_float(Txr8_current);
				write_lcd(' ');
				write_lcd(' ');
			
			}
			else if(disp_cntr==24)
			{
			
				cmd_lcd(0x80);
				display_lcd("INV1_A:");
				cmd_lcd(0xC0);
				lcd_float(Inv1_current);
				write_lcd(' ');
				write_lcd(' ');
			
			}
		
	}


/*


      sdata[1]=Comm_Inv1_vlt;
      sdata[2]=(Comm_Inv1_vlt>>8);
			
			sdata[3]=Comm_Inv2_vlt;
      sdata[4]=(Comm_Inv2_vlt>>8);
			
			sdata[5]=Comm_Cvt1_vlt;
      sdata[6]=(Comm_Cvt1_vlt>>8);
			
			sdata[7]=Comm_Cvt2_vlt;
      sdata[8]=(Comm_Cvt2_vlt>>8);
			
			sdata[9]=Comm_Txr1_vlt;
      sdata[10]=(Comm_Txr1_vlt>>8);
			
			sdata[11]=Comm_Txr2_vlt;
      sdata[12]=(Comm_Txr2_vlt>>8);
      			
      sdata[13]=Comm_Txr3_vlt;
      sdata[14]=(Comm_Txr3_vlt>>8);		

      sdata[15]=Comm_Txr4_vlt;
      sdata[16]=(Comm_Txr4_vlt>>8);	
			
			sdata[17]=Comm_Txr5_vlt;
      sdata[18]=(Comm_Txr5_vlt>>8);

      sdata[19]=Comm_Txr6_vlt;
      sdata[20]=(Comm_Txr6_vlt>>8);	
			
			sdata[21]=Comm_Txr7_vlt;
      sdata[22]=(Comm_Txr7_vlt>>8);	
			
			sdata[23]=Comm_Txr8_vlt;
      sdata[24]=(Comm_Txr8_vlt>>8);	
   
	    sdata[25]=Comm_Inv1_current;
      sdata[26]=(Comm_Inv1_current>>8);	
			
			sdata[27]=Comm_Inv2_current;
      sdata[28]=(Comm_Inv2_current>>8);	
			
			sdata[29]=Comm_Cvt1_current;
      sdata[30]=(Comm_Cvt1_current>>8);	
			
			sdata[31]=Comm_Cvt2_current;
      sdata[32]=(Comm_Cvt2_current>>8);	
			
			sdata[33]=Comm_Txr1_current;
      sdata[34]=(Comm_Txr1_current>>8);	
			
			sdata[35]=Comm_Txr2_current;
      sdata[36]=(Comm_Txr2_current>>8);	
			
			sdata[37]=Comm_Txr3_current;
      sdata[38]=(Comm_Txr3_current>>8);	
			
			sdata[39]=Comm_Txr4_current;
      sdata[40]=(Comm_Txr4_current>>8);	
			
			sdata[41]=Comm_Txr5_current;
      sdata[42]=(Comm_Txr5_current>>8);

      sdata[43]=Comm_Txr6_current;
      sdata[44]=(Comm_Txr6_current>>8);	
			  
			sdata[45]=Comm_Txr7_current;
      sdata[46]=(Comm_Txr7_current>>8);		
			
			sdata[47]=Comm_Txr8_current;
      sdata[48]=(Comm_Txr8_current>>8);	
			
			sdata[49]=0;
			sdata[50]=0;
	*/		
			
	
void Communication(void)
{
	unsigned char t,nb; 
 
	
	 if(flag10==1)   // 5msec TO 45msec flag10=1
   {
		   
       checksum=0;
       for(t=1;t<8;t++)
       checksum+=rdata[t];    
                            
       if(((0xff^checksum)==rdata[8])&&(rdata[0]==0X03))
		   { flag11=1; data11=rdata[1];}           
       else
       {flag10=0;}
			 
			
       for(t=0;(t<=8);t++)
       rdata[t]=0;    
			 
			 
    }
            
    if(flag11==1)
    { 
					
			sdata[0]=0x0a;  
			sdata[1]=Comm_Inv1_vlt;       //inv1 voltage
		
      sdata[2]=Comm_Inv2_vlt;       //inv2 voltage
			
			sdata[3]=Comm_Inv1_current;    //inv1 current
      sdata[4]=Comm_Inv2_current;    //inv2 current
			
			sdata[5]=Comm_Txr1_vlt;      ///Tx1 voltage
      sdata[6]=Comm_Txr2_vlt;      ///Tx2 voltage
			
			sdata[7]=Comm_Txr3_vlt;     ///Tx3 voltage
      sdata[8]=Comm_Txr4_vlt;     ///Tx4 voltage
			
			sdata[9]=Comm_Txr5_vlt;     ///Tx5 voltage
      sdata[10]=Comm_Txr6_vlt;    ///Tx6 voltage
			
			sdata[11]=Comm_Txr7_vlt;     ///Tx7 voltage
      sdata[12]=Comm_Txr8_vlt;     ///Tx8 voltage
      			
      sdata[13]=Comm_Txr1_current;   ///Tx1 Current
      sdata[14]=Comm_Txr2_current;	 ///Tx2 Current	

      sdata[15]=Comm_Txr3_current;   ///Tx3 Current
      sdata[16]=Comm_Txr4_current;	 ///Tx4 Current
			
			sdata[17]=Comm_Txr5_current;   ///Tx5 Current
      sdata[18]=Comm_Txr6_current;   ///Tx6 Current

      sdata[19]=Comm_Txr7_current;    ///Tx7 Current
      sdata[20]=Comm_Txr8_current;	  ///Tx8 Current
			
			sdata[21]=Comm_Cvt1_vlt;       ///Cvt1 voltage
      sdata[22]=Comm_Cvt2_vlt;	     ///Cvt2 voltage
			
			sdata[23]=Comm_Cvt1_current;   ///Cvt1 current
      sdata[24]=Comm_Cvt2_current;   ///Cvt2 current
   
	    sdata[25]=0;
      sdata[26]=0;	
			
			sdata[27]=0;
      sdata[28]=0;	
			
			sdata[29]=0;
      sdata[30]=0;	
			
			sdata[31]=0;
      sdata[32]=0;	
			
			sdata[33]=0;
      sdata[34]=0;	
			
			sdata[35]=0;
      sdata[36]=0;	
			
			sdata[37]=0;
      sdata[38]=0;	
			
			sdata[39]=0;
      sdata[40]=0;	
			
			sdata[41]=0;
      sdata[42]=0;	
			
			sdata[43]=0;
      sdata[44]=0;	
			
			sdata[45]=0;
      sdata[46]=0;	
			
			sdata[47]=0;
      sdata[48]=0;	
			
			sdata[49]=counter;
     
      sdata[50]=0; 
			
		/*	sdata[51]=230;  //inv1v
      sdata[52]=230;   //inv2v
      			
      sdata[53]=90; // inv1A
      sdata[54]=90;  //inv2A
      sdata[55]=110;  //S1V
			sdata[56]=110;  //S2V
      sdata[57]=110;  //S3V
      sdata[58]=110;  //S4V
      sdata[59]=110;  //S5V
			sdata[60]=110;  //S6V
			sdata[61]=110;  //S7V
      sdata[62]=110;  //S8V
      sdata[63]=90; // S1A
      sdata[64]=90; // S2A
			sdata[65]=90; // S3A
			sdata[66]=90; // S4A
      sdata[67]=90; // S5A
      sdata[68]=90; // S6A
      sdata[69]=90; // S7A
			sdata[70]=90; // S8A
			sdata[71]=230;  //CVT1V
      sdata[72]=230;  //CVT2V
      sdata[73]=90;  //CVT1A
      sdata[74]=90;  //CVT2A
			sdata[75]=10;
			sdata[76]=10;
      sdata[77]=10;
      sdata[78]=10;
      sdata[79]=10; 
			sdata[80]=10;
			sdata[81]=10;
      sdata[82]=10;
      sdata[83]=10;
      sdata[84]=10; 
			sdata[85]=data11;
			sdata[86]=data11;
      sdata[87]=data11;
      sdata[88]=data11;
      sdata[89]=data11; 
			sdata[90]=10;
	
			
			sdata[91]=data11;
      sdata[92]=data11;
      sdata[93]=data11;
      sdata[94]=data11; 
			sdata[95]=data11;
			sdata[96]=10;
      sdata[97]=10;
      sdata[98]=data11;
      sdata[99]=data11; 
			sdata[100]=0; 
			*/
		
			
       for(nb=1;(nb<50);nb++) 
       {
         sdata[50]+=sdata[nb];                
       }        
               
       sdata[50]^=0xff;
                                      
       RS485_ENABLE=1;     //rs 485 enable 
       TX_enable=1;
 
       ijk:
       if(EMPTY!=1)
       goto ijk;    
                    
       txdata=0;
       txdata|=0x100;
       txdata+=sdata[0];
       USART1_DR=txdata;   
                   
       flag11=0;
		
            
    }	 
		//data11=100;
  }	


void set(void)
{   
				
		cntr0=500;  //5 sec delay
    flg_set1=1;    
chk_set:             
  //  if(key==0x07) 
  //  ah_setting();
	
	  cmd_lcd(0x80);
    display_lcd("calib:");
		cmd_lcd(0xC0);
    display_lcd("        ");
	  
	
    if(key==0X03)
    {
       if(flg_set1)
       goto chk_set;   
       calibration();            
    }
		
}  

void up(void)
{   
	
   if(flag0==0)
	 {
	     flag0=1;
			disp_cntr++;
			if(disp_cntr==25) 
			disp_cntr=0x00;
			disp_para();  
   }	
	 
	 
	  
} 

void down(void)
{  
		//integer_lcd(8888);//////////////////
		
		if(flag0==0)
		{
				 flag0=1;
					disp_cntr--;
					if(disp_cntr==0x00)
					disp_cntr=24; 
					disp_para();
		}	
} 

 
void main(void)
{ 
  
   _asm("sim");
	 
   CLOCK_CONFG();  
   init_port();
   timer_sec();
	 ADC_INIT();
	 DMA_INIT();	 
	 
	 Lcd_Init();
	 
   delay(100);
	 
	 cmd_lcd(0x80);
	 display_lcd(" IPSVER4");
	 write_lcd(' ');
	 write_lcd(' ');
	 cmd_lcd(0xC0);
	 display_lcd("  ACDP");
	 write_lcd(' ');
	 write_lcd(' ');
	 delay1(30000);	

   EEPROM_Init();
	 
	 	 
	 key=0x0E;
	 key_deb2=0x0E;
	 key_deb1=0x0E;
	 key_deb0=0x0E;
	 
	 comm();
	 
	 read_calib();
	
  	 
	 DMA1_GCSR=1; // DMA enable
	 
	// MUL_A0=1;
	// MUL_A1=1;
	// MUL_A2=1;
	// MUL_A3=1;
	
   disp_cntr11=0;
   V0=0;V1=0;V2=0;V3=0;V4=0;V5=0;V6=0;V7=0;V8=0;V9=0;V10=0;V11=0;V12=0;V13=0;V14=0;V15=0; 
   flag5=0;
	 disp_cntr1=5000;
	 
	 disp_flag=1;disp_cntr=1;
	 
	 //MUL_A0=1; MUL_A1=0; MUL_A2=1; MUL_A3=0; 
	 
	 flag5=4; MUL_A0=0; MUL_A1=0; MUL_A2=1; MUL_A3=0;   //Y4
	 
	 LCD_CLEAR();
	 
	 disp_cntr=0;
	 
	 //ratio_lmt_t1=900;
	 
	 
   _asm("rim");

  while(1)
  {	

    if(m_sec_disp>200)
		{
			ac_calc();
								
			if(INV1_V<=10 || INV2_V<=10 || CVT1_V<=10 || CVT2_V<=10 || TXR1_V<=10 || TXR2_V<=10 ||
			TXR3_V<=10 || TXR4_V<=10 || TXR5_V<=10 || TXR6_V<=10) // || TXR7_V<=10 || TXR8_V<=10)
			{
				  FAULT_PIN=0;
		  }		
		  else
			{
				  FAULT_PIN=1;
		  }		
	  }	

    if(key==0x06)
			{up();flag1=1;}
		else if(key==0x05)
			{down();flag1=1;}
		else if(key==0x03)
			set();
		else if(key==0x07)
			flag0=0;
			
			Communication();

if(DISP>1500)
			{
				DISP=0;
				
			
			  disp_para();
				
					
			//	disp_cntr++;
			//	if(disp_cntr>23)
		  //		disp_cntr=0;
				
	
			
		  }		
			
			

				
		
  }
}