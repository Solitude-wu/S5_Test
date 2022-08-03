#line 1 "..\\src\\can_drv.c"
#line 1 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
 









 



















 








 




 



 

typedef enum IRQn {
     
    NonMaskableInt_IRQn         = -14,     
    HardFault_IRQn              = -13,     
    SVCall_IRQn                 = -5,      
    PendSV_IRQn                 = -2,      
    SysTick_IRQn                = -1,      

     
    BOD_IRQn                  = 0,         
    WDT_IRQn                  = 1,         
    EINT0_IRQn                = 2,         
    EINT1_IRQn                = 3,         
    GPAB_IRQn                 = 4,         
    GPCDEF_IRQn               = 5,         
    PWMA_IRQn                 = 6,         
    PWMB_IRQn                 = 7,         
    TMR0_IRQn                 = 8,         
    TMR1_IRQn                 = 9,         
    TMR2_IRQn                 = 10,        
    TMR3_IRQn                 = 11,        
    UART02_IRQn               = 12,        
    UART1_IRQn                = 13,        
    SPI0_IRQn                 = 14,        
    SPI1_IRQn                 = 15,        
    SPI2_IRQn                 = 16,        
    SPI3_IRQn                 = 17,        
    I2C0_IRQn                 = 18,        
    I2C1_IRQn                 = 19,        
    CAN0_IRQn                 = 20,        
    CAN1_IRQn                 = 21,        
    SC012_IRQn                = 22,        
    USBD_IRQn                 = 23,        
    PS2_IRQn                  = 24,        
    ACMP_IRQn                 = 25,        
    PDMA_IRQn                 = 26,        
    I2S_IRQn                  = 27,        
    PWRWU_IRQn                = 28,        
    ADC_IRQn                  = 29,        
    IRC_IRQn                  = 30,        
    RTC_IRQn                  = 31         
} IRQn_Type;






 

 





   


#line 1 "..\\Library\\CMSIS\\Include\\core_cm0.h"
 




















 













 












 




 


 

 













#line 89 "..\\Library\\CMSIS\\Include\\core_cm0.h"


 







#line 114 "..\\Library\\CMSIS\\Include\\core_cm0.h"

#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 116 "..\\Library\\CMSIS\\Include\\core_cm0.h"
#line 1 "..\\Library\\CMSIS\\Include\\core_cmInstr.h"
 




















 






 


 



 


 









 







 







 






 








 







 







 









 









 
__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 
__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 



#line 268 "..\\Library\\CMSIS\\Include\\core_cmInstr.h"



#line 619 "..\\Library\\CMSIS\\Include\\core_cmInstr.h"

   

   

#line 117 "..\\Library\\CMSIS\\Include\\core_cm0.h"
#line 1 "..\\Library\\CMSIS\\Include\\core_cmFunc.h"
 




















 






 

 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 260 "..\\Library\\CMSIS\\Include\\core_cmFunc.h"


#line 296 "..\\Library\\CMSIS\\Include\\core_cmFunc.h"


#line 615 "..\\Library\\CMSIS\\Include\\core_cmFunc.h"

 

   

#line 118 "..\\Library\\CMSIS\\Include\\core_cm0.h"








 
#line 143 "..\\Library\\CMSIS\\Include\\core_cm0.h"

 






 
#line 159 "..\\Library\\CMSIS\\Include\\core_cm0.h"

 










 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[1];                  
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];                  
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];                  
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];                  
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];                    
}  NVIC_Type;

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
       uint32_t RESERVED0;
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];                   
  volatile uint32_t SHCSR;                    
} SCB_Type;

 















 



























 















 









 






 



 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 








 
 






 

 










 









 

 



 




 

 
 










 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2)));  }  
  else {
    return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2)));  }  
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (1UL << 2));
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFUL << 0))  return (1);             

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (ticks & (0xFFFFFFUL << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 








   

#line 111 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\system_NUC230_240.h"
 









 






 
 
 

 




 






extern uint32_t SystemCoreClock;     
extern uint32_t CyclesPerUs;         
extern uint32_t PllClock;            









 
extern void SystemInit(void);










 
extern void SystemCoreClockUpdate(void);





#line 112 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"


#pragma anon_unions



 



 




 



 


typedef struct
{














































 

    volatile uint32_t CMPCR[2];       
    volatile uint32_t CMPSR;          

} ACMP_T;





 



 















 











   
   



 



 


typedef struct
{













































































































































































































 

    volatile const  uint32_t ADDR[8];        
    volatile uint32_t ADCR;           
    volatile uint32_t ADCHER;         
    volatile uint32_t ADCMPR[2];      
    volatile uint32_t ADSR;           
    volatile const  uint32_t RESERVE0[3];  
    volatile const  uint32_t ADPDMA;         

} ADC_T;





 



 









 






























 






 


















 





















 


   
   



 



 



typedef struct
{
























































































































































































































































 

    volatile uint32_t CREQ;           
    volatile uint32_t CMASK;          
    volatile uint32_t MASK1;          
    volatile uint32_t MASK2;          
    volatile uint32_t ARB1;           
    volatile uint32_t ARB2;           
    volatile uint32_t MCON;           
    volatile uint32_t DAT_A1;         
    volatile uint32_t DAT_A2;         
    volatile uint32_t DAT_B1;         
    volatile uint32_t DAT_B2;         
    volatile const  uint32_t RESERVE[13];   

} CAN_IF_T;





typedef struct
{




















































































































































































































































 

    volatile uint32_t CON;            
    volatile uint32_t STATUS;         
    volatile uint32_t ERR;            
    volatile uint32_t BTIME;          
    volatile uint32_t IIDR;           
    volatile uint32_t TEST;           
    volatile uint32_t BRPE;           
    volatile const  uint32_t RESERVE0[1];  
    volatile CAN_IF_T IF[2];          
    volatile const  uint32_t RESERVE1[8];  
    volatile uint32_t TXREQ1;         
    volatile uint32_t TXREQ2;         
    volatile const  uint32_t RESERVE2[6];  
    volatile uint32_t NDAT1;          
    volatile uint32_t NDAT2;          
    volatile const  uint32_t RESERVE3[6];  
    volatile uint32_t IPND1;          
    volatile uint32_t IPND2;          
    volatile const  uint32_t RESERVE4[6];  
    volatile uint32_t MVLD1;          
    volatile uint32_t MVLD2;          
    volatile uint32_t WU_EN;          
    volatile uint32_t WU_STATUS;      

} CAN_T;






 


 





















 


















 









 












 



 















 



 






 
























 



 









 



 












 






























 






 






 






 






 



 



 



 



 



 



 



 



 



 


   
   


 



 



typedef struct
{





















































































































































































































































































































































































































































































































































 

    volatile uint32_t PWRCON;         
    volatile uint32_t AHBCLK;         
    volatile uint32_t APBCLK;         
    volatile uint32_t CLKSTATUS;      
    volatile uint32_t CLKSEL0;        
    volatile uint32_t CLKSEL1;        
    volatile uint32_t CLKDIV;         
    volatile uint32_t CLKSEL2;        
    volatile uint32_t PLLCON;         
    volatile uint32_t FRQDIV;         
    volatile uint32_t RESERVE[2];   
    volatile uint32_t APBCLK1;        
    volatile uint32_t CLKSEL3;        
    volatile uint32_t CLKDIV1;        

} CLK_T;





 


 































 









 

















































































 









 






















 






 







































 






































 









 












 









 





















 











   
   



 



 


typedef struct
{








































































































































































 

    volatile uint32_t CTL;            
    volatile uint32_t DMASAR;         
    volatile const  uint32_t RESERVED0;    
    volatile uint32_t DMABCR ;        
    volatile const  uint32_t RESERVED1;    
    volatile const  uint32_t DMACSAR;        
    volatile const  uint32_t RESERVED2;    
    volatile const  uint32_t DMACBCR;        
    volatile uint32_t DMAIER ;        
    volatile uint32_t DMAISR;         
    volatile const  uint32_t RESERVED3[22];
    volatile uint32_t WDATA;          
    volatile uint32_t SEED;           
    volatile const  uint32_t CHECKSUM;       

} CRC_T;





 


 



























 



 



 



 



 






 






 



 



 


   
   


 



 


typedef struct
{































































 

    volatile uint32_t EBICON;         
    volatile uint32_t EXTIME;         
    volatile uint32_t EBICON2;        

} EBI_T;






 

 












 












 








   
   

 



 



typedef struct
{























































































































































 

    volatile uint32_t ISPCON;         
    volatile uint32_t ISPADR;         
    volatile uint32_t ISPDAT;         
    volatile uint32_t ISPCMD;         
    volatile uint32_t ISPTRG;         
    volatile const  uint32_t DFBADR;         
    volatile uint32_t FATCON;         
    volatile const  uint32_t RESERVED[9];  
    volatile uint32_t ISPSTA;         

} FMC_T;





 


 
























 



 



 









 



 



 
























   
   


 



 



typedef struct
{
















































































































































 

    volatile uint32_t PMD;            
    volatile uint32_t OFFD;           
    volatile uint32_t DOUT;           
    volatile uint32_t DMASK;          
    volatile const  uint32_t PIN;            
    volatile uint32_t DBEN;           
    volatile uint32_t IMD;            
    volatile uint32_t IEN;            
    volatile uint32_t ISRC;           

} GPIO_T;





typedef struct
{

































 

    volatile uint32_t DBNCECON;       

} GPIO_DBNCECON_T;





 


 
















































 



 



 



 



 



 



 






 



 








   
   



 



 


typedef struct
{




















































































































































































































 

    volatile uint32_t I2CON;          
    volatile uint32_t I2CADDR0;       
    volatile uint32_t I2CDAT;         
    volatile const  uint32_t I2CSTATUS;      
    volatile uint32_t I2CLK;          
    volatile uint32_t I2CTOC;         
    volatile uint32_t I2CADDR1;       
    volatile uint32_t I2CADDR2;       
    volatile uint32_t I2CADDR3;       
    volatile uint32_t I2CADM0;        
    volatile uint32_t I2CADM1;        
    volatile uint32_t I2CADM2;        
    volatile uint32_t I2CADM3;        
    volatile const  uint32_t RESERVED0[2]; 
    volatile uint32_t I2CWKUPCON;     
    volatile uint32_t I2CWKUPSTS;     

} I2C_T;





 



 


















 






 



 



 



 









 



 



 


   
   



 



 


typedef struct
{


























































































































































































































































































 

    volatile uint32_t CON;            
    volatile uint32_t CLKDIV;         
    volatile uint32_t IE;             
    volatile uint32_t STATUS;         
    volatile  uint32_t TXFIFO;         
    volatile const  uint32_t RXFIFO;         

} I2S_T;






 


 






















































 






 

























 
























































   
   



 



 


typedef struct
{







































































































































 

    volatile uint32_t CSR;            
    volatile uint32_t SAR;            
    volatile uint32_t DAR;            
    volatile uint32_t BCR;            
    volatile const  uint32_t POINT;          
    volatile const  uint32_t CSAR;           
    volatile const  uint32_t CDAR;           
    volatile const  uint32_t CBCR;           
    volatile uint32_t IER;            
    volatile uint32_t ISR;            
    volatile const  uint32_t RESERVE[22];  
    volatile const  uint32_t SBUF;           

} PDMA_T;





typedef struct
{


































































































































































































 

    volatile uint32_t GCRCSR;         
    volatile uint32_t PDSSR0;         
    volatile uint32_t PDSSR1;         
    volatile uint32_t GCRISR;         
    volatile uint32_t PDSSR2;         

} PDMA_GCR_T;






 


 





















 



 



 




 






 






 






























 
























 















 

































 





   
   


 



 


typedef struct
{




































































































































































 

    volatile uint32_t PS2CON;         
    volatile uint32_t PS2TXDATA0;     
    volatile uint32_t PS2TXDATA1;     
    volatile uint32_t PS2TXDATA2;     
    volatile uint32_t PS2TXDATA3;     
    volatile uint32_t PS2RXDATA;      
    volatile uint32_t PS2STATUS;      
    volatile uint32_t PS2INTID;       

} PS2_T;






 


 



























 



 



























 





   
   


 



 



typedef struct
{












































































































































































































































































































































































































































































































































































































































































































































































































 

    volatile uint32_t PPR;            
    volatile uint32_t CSR;            
    volatile uint32_t PCR;            
    volatile uint32_t CNR0;           
    volatile uint32_t CMR0;           
    volatile const  uint32_t PDR0;           
    volatile uint32_t CNR1;           
    volatile uint32_t CMR1;           
    volatile const  uint32_t PDR1;           
    volatile uint32_t CNR2;           
    volatile uint32_t CMR2;           
    volatile const  uint32_t PDR2;           
    volatile uint32_t CNR3;           
    volatile uint32_t CMR3;           
    volatile const  uint32_t PDR3;           
    volatile uint32_t PBCR;           
    volatile uint32_t PIER;           
    volatile uint32_t PIIR;           
    volatile const  uint32_t RESERVE1[2];  
    volatile uint32_t CCR0;           
    volatile uint32_t CCR2;           
    volatile uint32_t CRLR0;          
    volatile uint32_t CFLR0;          
    volatile uint32_t CRLR1;          
    volatile uint32_t CFLR1;          
    volatile uint32_t CRLR2;          
    volatile uint32_t CFLR2;          
    volatile uint32_t CRLR3;          
    volatile uint32_t CFLR3;          
    volatile uint32_t CAPENR;         
    volatile uint32_t POE;            
    volatile uint32_t TCON;           
    volatile uint32_t TSTATUS;        
    volatile uint32_t SYNCBUSY0;      
    volatile uint32_t SYNCBUSY1;      
    volatile uint32_t SYNCBUSY2;      
    volatile uint32_t SYNCBUSY3;      

} PWM_T;





 


 












 












 




























































 



 



 



 



 































 
























 










































 










































 



 



 












 












 













 













 



 



 



 


   
   


 



 


typedef struct
{

































































































































































































































 

    volatile uint32_t INIR;           
    volatile uint32_t AER;            
    volatile uint32_t FCR;            
    volatile uint32_t TLR;            
    volatile uint32_t CLR;            
    volatile uint32_t TSSR;           
    volatile uint32_t DWR;            
    volatile uint32_t TAR;            
    volatile uint32_t CAR;            
    volatile const  uint32_t LIR;            
    volatile uint32_t RIER;           
    volatile uint32_t RIIR;           
    volatile uint32_t TTR;            
    volatile const  uint32_t RESERVED[2];  
    volatile uint32_t SPRCTL;         
    volatile uint32_t SPR[20];        

} RTC_T;





 



 






 






 






 


















 


















 



 



 


















 


















 



 









 









 



 

















   
   


 



 



typedef struct
{


    
























































































































































































































































































































































































































































































































































































































































































 
    
    union {
    volatile const  uint32_t RBR;            
    volatile  uint32_t THR;            
    };
    volatile uint32_t CTL;            
    volatile uint32_t ALTCTL;         
    volatile uint32_t EGTR;           
    volatile uint32_t RFTMR;          
    volatile uint32_t ETUCR;          
    volatile uint32_t IER;            
    volatile uint32_t ISR;            
    volatile uint32_t TRSR;           
    volatile uint32_t PINCSR;         
    volatile uint32_t TMR0;           
    volatile uint32_t TMR1;           
    volatile uint32_t TMR2;           
    volatile uint32_t UACTL;          
    volatile const  uint32_t TDRA;           
    volatile const  uint32_t TDRB;           

} SC_T;





 


 



 



 













































 










































 



 



 






 

































 


































 



















































 







































 






 






 






 












 



 






   
   


 



 



typedef struct
{




























































































































































































































































































































































































































 

    volatile uint32_t CNTRL;          
    volatile uint32_t DIVIDER;        
    volatile uint32_t SSR;            
    volatile const  uint32_t RESERVE0;     
    volatile const  uint32_t RX[2];          
    volatile const  uint32_t RESERVE1;     
    volatile const  uint32_t RESERVE2;     
    volatile  uint32_t TX[2];          
    volatile const  uint32_t RESERVE3;     
    volatile const  uint32_t RESERVE4;     
    volatile const  uint32_t RESERVE5;     
    volatile uint32_t VARCLK;         
    volatile uint32_t DMA;            
    volatile uint32_t CNTRL2;         
    volatile uint32_t FIFO_CTL;       
    volatile uint32_t STATUS;         

} SPI_T;






 


 






















































 






 















 









 
























 
























 



































   
   




 



 



typedef struct
{
































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































 

    volatile const  uint32_t PDID;           
    volatile uint32_t RSTSRC;         
    volatile uint32_t IPRSTC1;        
    volatile uint32_t IPRSTC2;        
    volatile uint32_t IPRSTC3;        
    volatile const  uint32_t RESERVE0;     
    volatile uint32_t BODCR;          
    volatile uint32_t TEMPCR;         
    volatile const  uint32_t RESERVE1;     
    volatile uint32_t PORCR;          
    volatile const  uint32_t RESERVE2[2];  
    volatile uint32_t GPA_MFP;        
    volatile uint32_t GPB_MFP;        
    volatile uint32_t GPC_MFP;        
    volatile uint32_t GPD_MFP;        
    volatile uint32_t GPE_MFP;        
    volatile uint32_t GPF_MFP;        
    volatile const  uint32_t RESERVE3[2];  
    volatile uint32_t ALT_MFP;        
    volatile const  uint32_t RESERVE4;     
    volatile uint32_t ALT_MFP1;       
    volatile uint32_t ALT_MFP2;       
    volatile const  uint32_t RESERVE5[8];  
    volatile uint32_t IRCTRIMCTL;     
    volatile uint32_t IRCTRIMIEN;     
    volatile uint32_t IRCTRIMINT;     
    volatile const  uint32_t RESERVE6[29]; 
    volatile uint32_t REGWRPROT;      

} GCR_T;





 


 





















 












 





































































 









 





















 



 



 







 






 






 






 












 















 


































































 













































 


















 












 






 









 





   



typedef struct
{


























































































 

    volatile const  uint32_t IRQSRC[32];    
    volatile uint32_t NMISEL;        
    volatile uint32_t MCUIRQ;        
    volatile uint32_t MCUIRQCR;       

} GCR_INT_T;





 


 



 






 



 



   
   


 



 


typedef struct
{


















































































































































 

    volatile uint32_t TCSR;           
    volatile uint32_t TCMPR;          
    volatile uint32_t TISR;           
    volatile const  uint32_t TDR;            
    volatile const  uint32_t TCAP;           
    volatile uint32_t TEXCON;         
    volatile uint32_t TEXISR;         

} TIMER_T;





 


 






























 



 






 



 



 





















 


   
   


 



 



typedef struct
{






















































































































































































































































































































































































































































































































































































































































































































 

    union {
        volatile uint32_t DATA;           
        volatile uint32_t THR;            
        volatile uint32_t RBR;            
    };
    volatile uint32_t IER;            
    volatile uint32_t FCR;            
    volatile uint32_t LCR;            
    volatile uint32_t MCR;            
    volatile uint32_t MSR;            
    volatile uint32_t FSR;            
    volatile uint32_t ISR;            
    volatile uint32_t TOR;            
    volatile uint32_t BAUD;           
    volatile uint32_t IRCR;           
    volatile uint32_t ALT_CSR;        
    volatile uint32_t FUN_SEL;        
    volatile uint32_t LIN_CTL;        
    volatile uint32_t LIN_SR;         
    

} UART_T;






 


 



 



 







































 















 


















 









 










 







































 



































































 






 












 









 
























 



 










































 


















   
   


 



 


typedef struct
{










































































 

    volatile uint32_t BUFSEG;         
    volatile uint32_t MXPLD;          
    volatile uint32_t CFG;            
    volatile uint32_t CFGP;           

} USBD_EP_T;






typedef struct
{








































































































































































































































 

    volatile uint32_t INTEN;          
    volatile uint32_t INTSTS;         
    volatile uint32_t FADDR;          
    volatile const  uint32_t EPSTS;          
    volatile uint32_t ATTR;           
    volatile const  uint32_t FLDET;          
    volatile uint32_t STBUFSEG;       
    volatile const  uint32_t RESERVE1[29];  
    volatile uint32_t DRVSE0;         
    volatile const  uint32_t RESERVE2[283];
        USBD_EP_T EP[8];          

} USBD_T;





 


 


















 







































 



 





















 






























 



 



 



 



 















 






 


   
   


 



 


typedef struct
{


















































































 

    volatile uint32_t WTCR;           
    volatile uint32_t WTCRALT;        

} WDT_T;






 



 






























 


   
   


 



 


typedef struct
{













































































 

    volatile uint32_t WWDTRLD;        
    volatile uint32_t WWDTCR;         
    volatile uint32_t WWDTSR;         
    volatile const  uint32_t WWDTCVR;        

} WWDT_T;





 



 



 















 






 


   
   
   


 
 
 



 
 






 
#line 12021 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"













































#line 12075 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"















   

 
 
 




 
#line 12107 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"














































#line 12162 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"














   











 

typedef volatile unsigned char  vu8;        
typedef volatile unsigned short vu16;       
typedef volatile unsigned long  vu32;       





 







 







 








 







 








 







 







 






 








 







 








 







 







 






 


   







 













 
#line 12370 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"

 










   

   


 
 
 
#line 1 "..\\Library\\StdDriver\\inc\\sys.h"
 









 










 



 



 


 
 
 
#line 64 "..\\Library\\StdDriver\\inc\\sys.h"


 
 
 
#line 75 "..\\Library\\StdDriver\\inc\\sys.h"


 
 
 







 








































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































   



 







 







 







 







 








 









 







 







 







 











 








 








 








 








 








 








 








 







 







 







 







 














 









 
static __inline void SYS_LockReg(void)
{
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0;
}








 
static __inline void SYS_UnlockReg(void)
{
    while(((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT != (1ul << 0))
    {
        ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0x59;
        ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0x16;
        ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0x88;
    }
}


void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t  SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);


   

   

   








 
#line 12391 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\adc.h"
 









 



#line 1 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
 









 



















 

#line 12414 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"

   


#line 16 "..\\Library\\StdDriver\\inc\\adc.h"









 



 



 
 
 
 
























 
 
 




 
 
 
#line 77 "..\\Library\\StdDriver\\inc\\adc.h"

 
 
 




 
 
 




 
 
 





 
 
 



 
 
 





   



 











 








 







 








 












 












 








 









 









 








 







 
















 
#line 255 "..\\Library\\StdDriver\\inc\\adc.h"






 
















 
#line 288 "..\\Library\\StdDriver\\inc\\adc.h"






 










 










 







 








 


void ADC_Open(ADC_T *adc,
              uint32_t u32InputMode,
              uint32_t u32OpMode,
              uint32_t u32ChMask);
void ADC_Close(ADC_T *adc);
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param);
void ADC_DisableHWTrigger(ADC_T *adc);
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask);



   

   

   







 
#line 12392 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\can.h"
 









 



#line 16 "..\\Library\\StdDriver\\inc\\can.h"









 



 



 
 
 
 



 
 
 



 
 
 



   

 
 
 
typedef struct
{

    uint8_t   IdType;
    uint8_t   FrameType;
    uint8_t   DLC;
    uint32_t  Id;
    uint8_t   Data[8];
#line 72 "..\\Library\\StdDriver\\inc\\can.h"
} STR_CANMSG_T;

 
 
 
typedef struct
{
    uint8_t   u8Xtd;
    uint8_t   u8Dir;
    uint32_t  u32Id;
    uint8_t   u8IdType;
} STR_CANMASK_T;






 










 











 










 










 












 



 
 
 
uint32_t CAN_SetBaudRate(CAN_T *tCAN, uint32_t u32BaudRate);
uint32_t CAN_Open(CAN_T *tCAN, uint32_t u32BaudRate, uint32_t u32Mode);
void CAN_Close(CAN_T *tCAN);
void CAN_CLR_INT_PENDING_BIT(CAN_T *tCAN, uint8_t u32MsgNum);
void CAN_EnableInt(CAN_T *tCAN, uint32_t u32Mask);
void CAN_DisableInt(CAN_T *tCAN, uint32_t u32Mask);
int32_t CAN_Transmit(CAN_T *tCAN, uint32_t u32MsgNum , STR_CANMSG_T* pCanMsg);
int32_t CAN_Receive(CAN_T *tCAN, uint32_t u32MsgNum , STR_CANMSG_T* pCanMsg);
int32_t CAN_SetMultiRxMsg(CAN_T *tCAN, uint32_t u32MsgNum , uint32_t u32MsgCount, uint32_t u32IDType, uint32_t u32ID);
int32_t CAN_SetRxMsg(CAN_T *tCAN, uint32_t u32MsgNum , uint32_t u32IDType, uint32_t u32ID);
int32_t CAN_SetRxMsgAndMsk(CAN_T *tCAN, uint32_t u32MsgNum , uint32_t u32IDType, uint32_t u32ID, uint32_t u32IDMask);
int32_t CAN_SetTxMsg(CAN_T *tCAN, uint32_t u32MsgNum , STR_CANMSG_T* pCanMsg);
int32_t CAN_TriggerTxMsg(CAN_T  *tCAN, uint32_t u32MsgNum);


   

   

   







 
#line 12393 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\fmc.h"
 









 



#line 16 "..\\Library\\StdDriver\\inc\\fmc.h"









 



 



 


 
 
 







 
 
 



 
 
 
#line 62 "..\\Library\\StdDriver\\inc\\fmc.h"


   



 

 
 
 









 












 













 













 














 











 













 












 













 















 














 



 
 
 











 
static __inline void FMC_Write(uint32_t u32addr, uint32_t u32data)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x21;    
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32addr;               
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT = u32data;               
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                   
    __isb(0xF);                             
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);                  
}










 
static __inline uint32_t FMC_Read(uint32_t u32addr)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x00;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32addr;          
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;              
    __isb(0xF);                        
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);             

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}













 
static __inline int32_t FMC_Erase(uint32_t u32addr)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x22;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32addr;                
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                    
    __isb(0xF);                              
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);                   

     
    if(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCON & (1ul << 6))
    {
        ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCON |= (1ul << 6);
        return -1;
    }
    return 0;
}










 
static __inline uint32_t FMC_ReadUID(uint8_t u8index)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x04;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = (u8index << 2);       
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                  
    __isb(0xF);                            
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);                 

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}











 
static __inline uint32_t FMC_ReadCID(void)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x0B;            
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = 0x0;                            
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = (1ul << 0);           
    __isb(0xF);                                      
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG & (1ul << 0)) ;   

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}










 
static __inline uint32_t FMC_ReadPID(void)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x0C;           
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = 0x04;                          
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = (1ul << 0);          
    __isb(0xF);                                     
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG & (1ul << 0));   

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}










 
static __inline uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x04;           
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = (0x04 * u32Index) + 0x10;      
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = (1ul << 0);          
    __isb(0xF);                                     
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG & (1ul << 0));   

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}















 
static __inline void FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x2e;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32PageAddr;        
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                
    __isb(0xF);                          
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);               
}














 
static __inline uint32_t FMC_GetVECMAP(void)
{
    return (((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPSTA & (0xFFFul << 9));
}

extern void FMC_Open(void);
extern void FMC_Close(void);
extern void FMC_EnableAPUpdate(void);
extern void FMC_DisableAPUpdate(void);
extern void FMC_EnableConfigUpdate(void);
extern void FMC_DisableConfigUpdate(void);
extern void FMC_EnableLDUpdate(void);
extern void FMC_DisableLDUpdate(void);
extern int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);
extern void FMC_SetBootSource(int32_t i32BootSrc);
extern int32_t FMC_GetBootSource(void);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);

   

   

   








#line 12394 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\gpio.h"
 









 











 



 



 


 
 
 





 
 
 






 
 
 



 
 
 






#line 82 "..\\Library\\StdDriver\\inc\\gpio.h"















 
#line 183 "..\\Library\\StdDriver\\inc\\gpio.h"


   




 












 













 













 













 













 













 













 














 



















 










 











 










 















 














 
















 














 



void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin);


   

   

   







 
#line 12395 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\i2c.h"
 









 



#line 16 "..\\Library\\StdDriver\\inc\\i2c.h"









 



 



 

 
 
 
#line 47 "..\\Library\\StdDriver\\inc\\i2c.h"




   



 









 










 










 










 











 










 











 











 










 


 
 
 









 
static __inline void I2C_STOP(I2C_T *i2c)
{
    (i2c)->I2CON |= ((1ul << 3) | (1ul << 4));
    while((i2c)->I2CON & (1ul << 4));
}

void I2C_ClearTimeoutFlag(I2C_T *i2c);
void I2C_Close(I2C_T *i2c);
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void I2C_DisableInt(I2C_T *i2c);
void I2C_EnableInt(I2C_T *i2c);
uint32_t I2C_GetBusClockFreq(I2C_T *i2c);
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock);
uint32_t I2C_GetIntFlag(I2C_T *i2c);
uint32_t I2C_GetStatus(I2C_T *i2c);
uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock);
uint8_t I2C_GetData(I2C_T *i2c);
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode);
void I2C_SetSlaveAddrMask(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask);
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout);
void I2C_DisableTimeout(I2C_T *i2c);
void I2C_EnableWakeup(I2C_T *i2c);
void I2C_DisableWakeup(I2C_T *i2c);
void I2C_SetData(I2C_T *i2c, uint8_t u8Data);

   

   

   

#line 12396 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\pwm.h"
 








 











 



 



 
#line 43 "..\\Library\\StdDriver\\inc\\pwm.h"
 
 
 






   




 











 
#line 79 "..\\Library\\StdDriver\\inc\\pwm.h"









 










 













 

















 













 














 
















 
#line 186 "..\\Library\\StdDriver\\inc\\pwm.h"


uint32_t PWM_ConfigCaptureChannel(PWM_T *pwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32UnitTimeNsec,
                                  uint32_t u32CaptureEdge);
uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,
                                 uint32_t u32ChannelNum,
                                 uint32_t u32Frequncy,
                                 uint32_t u32DutyCycle);
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void PWM_DisableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t PWM_GetADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_DisableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_ClearCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t PWM_GetCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void PWM_DisableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void PWM_DisablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);



   

   

   







 
#line 12397 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\spi.h"
 









 











 



 



 














#line 52 "..\\Library\\StdDriver\\inc\\spi.h"









   




 






 







 







 







 







 







 







 








 







 







 







 







 







 








 








 








 







 







 








 








 










 







 







 







 







 







 







 










 







 







 








 








 









 




 
uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_ClearRxFIFO(SPI_T *spi);
void SPI_ClearTxFIFO(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void SPI_EnableFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
void SPI_DisableFIFO(SPI_T *spi);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetIntFlag(SPI_T *spi, uint32_t u32Mask);
void SPI_ClearIntFlag(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetStatus(SPI_T *spi, uint32_t u32Mask);





 



 



 




#line 12398 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\crc.h"
 








 











 



 



 
 
 
 





 
 
 





 
 
 




   




 









 










 










 










 













 










 










 



 
void CRC_Open(uint32_t u32Mode, uint32_t u32Attribute, uint32_t u32Seed, uint32_t u32DataLen);
void CRC_StartDMATransfer(uint32_t u32SrcAddr, uint32_t u32ByteCount);
uint32_t CRC_GetChecksum(void);

   

   

   







 
#line 12399 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\timer.h"
 








 











 



 



 

#line 43 "..\\Library\\StdDriver\\inc\\timer.h"

   




 










 












 











 










 
static __inline void TIMER_Start(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 30);
}









 
static __inline void TIMER_Stop(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 30);
}










 
static __inline void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 23);
}









 
static __inline void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 23);
}









 
static __inline void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 6);
}









 
static __inline void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 6);
}









 
static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 7);
}









 
static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 7);
}









 
static __inline void TIMER_EnableInt(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 29);
}









 
static __inline void TIMER_DisableInt(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 29);
}









 
static __inline void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 5);
}









 
static __inline void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 5);
}










 
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return (timer->TISR & (1ul << 0) ? 1 : 0);
}









 
static __inline void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->TISR = (1ul << 0);
}










 
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return timer->TEXISR;
}









 
static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->TEXISR = (1ul << 0);
}










 
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->TISR & (1ul << 1) ? 1 : 0);
}









 
static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->TISR = (1ul << 1);
}









 
static __inline uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->TCAP;
}









 
static __inline uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->TDR;
}

uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);

   

   

   







 
#line 12400 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\wdt.h"
 








 











 



 



 
 
 
 
#line 42 "..\\Library\\StdDriver\\inc\\wdt.h"

 
 
 





   




 









 










 










 











 











 











 












 










 
static __inline void WDT_Close(void)
{
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x4000))->WTCR = 0;
    return;
}









 
static __inline void WDT_EnableInt(void)
{
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x4000))->WTCR |= (1ul << 6);
    return;
}









 
static __inline void WDT_DisableInt(void)
{
    
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x4000))->WTCR &= ~((1ul << 6) | (1ul << 2) | (1ul << 3) | (1ul << 5));
    return;
}

void WDT_Open(uint32_t u32TimeoutInterval, uint32_t u32ResetDelay, uint32_t u32EnableReset, uint32_t u32EnableWakeup);

   

   

   







 
#line 12401 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\wwdt.h"
 








 











 



 



 
 
 
 
#line 50 "..\\Library\\StdDriver\\inc\\wwdt.h"



   




 









 










 











 











 










 












 


void WWDT_Open(uint32_t u32PreScale, uint32_t u32CmpValue, uint32_t u32EnableInt);

   

   

   







 
#line 12402 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\rtc.h"
 








 











 



 



 















#line 54 "..\\Library\\StdDriver\\inc\\rtc.h"

#line 62 "..\\Library\\StdDriver\\inc\\rtc.h"







   




 


 
#line 92 "..\\Library\\StdDriver\\inc\\rtc.h"
typedef struct
{
    uint16_t u32Year;            
    uint8_t u32Month;           
    uint8_t u32Day;             
    uint8_t u32DayOfWeek;       
    uint8_t u32Hour;            
    uint8_t u32Minute;          
    uint8_t u32Second;          
    uint8_t u32TimeScale;       
    uint8_t u32AmPm;            
} S_RTC_TIME_DATA_T;

   




 










 










 










 











 











 










 











 












 













 










 
static __inline void RTC_WaitAccessEnable(void)
{
     
    while((((RTC_T *) ((( uint32_t)0x40000000) + 0x08000))->AER & (1ul << 16)) == (1ul << 16));
    ((RTC_T *) ((( uint32_t)0x40000000) + 0x08000))->AER = 0x0000A965UL;

     
    while((((RTC_T *) ((( uint32_t)0x40000000) + 0x08000))->AER & (1ul << 16)) == 0x0);
}

void RTC_Open(S_RTC_TIME_DATA_T *sPt);
void RTC_Close(void);
void RTC_32KCalibration(int32_t i32FrequencyX100);
void RTC_GetDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_GetAlarmDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_SetDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_SetAlarmDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_SetDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day, uint32_t u32DayOfWeek);
void RTC_SetTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
void RTC_SetAlarmDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day);
void RTC_SetAlarmTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
uint32_t RTC_GetDayOfWeek(void);
void RTC_SetTickPeriod(uint32_t u32TickSelection);
void RTC_EnableInt(uint32_t u32IntFlagMask);
void RTC_DisableInt(uint32_t u32IntFlagMask);
void RTC_EnableSpareRegister(void);
void RTC_DisableSpareRegister(void);
void RTC_EnableSnooperDetection(uint32_t u32PinCondition);
void RTC_DisableSnooperDetection(void);

   

   

   







 
#line 12403 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\uart.h"
 









 












 



 



 

 
 
 





 
 
 











 
 
 
















 
 
 



 
 
 



 
 
 





 
 
 
#line 114 "..\\Library\\StdDriver\\inc\\uart.h"

 
 
 




   




 











 











 












 










 












 












 











 











 











 












 











 












 












 




















 



















 



































 











 
static __inline void UART_CLEAR_RTS(UART_T* uart)
{
    (uart)->MCR |= (1ul << 9);
    (uart)->MCR &= ~(1ul << 1);
}









 
static __inline void UART_SET_RTS(UART_T* uart)
{
    (uart)->MCR |= (1ul << 9) | (1ul << 1);
}










 











 



void UART_ClearIntFlag(UART_T* uart , uint32_t u32InterruptFlag);
void UART_Close(UART_T* uart);
void UART_DisableFlowCtrl(UART_T* uart);
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag);
void UART_EnableFlowCtrl(UART_T* uart);
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag);
void UART_Open(UART_T* uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction);
void UART_SelectRS485Mode(UART_T* uart, uint32_t u32Mode, uint32_t u32Addr);
void UART_SelectLINMode(UART_T* uart, uint32_t u32Mode, uint32_t u32BreakLength);
uint32_t UART_Write(UART_T* uart, uint8_t *pu8TxBuf, uint32_t u32WriteBytes);


   

   

   







 

#line 12404 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\i2s.h"
 








 



#line 15 "..\\Library\\StdDriver\\inc\\i2s.h"








 



 



 





 



 



 



 
#line 58 "..\\Library\\StdDriver\\inc\\i2s.h"

#line 67 "..\\Library\\StdDriver\\inc\\i2s.h"

 



 



   



 
 
 
 








 
static __inline void I2S_ENABLE_TX_ZCD(I2S_T *i2s, uint32_t u32ChMask)
{
    if(u32ChMask == 0)
        i2s->CON |= (1ul << 16);
    else
        i2s->CON |= (1ul << 17);
}









 
static __inline void I2S_DISABLE_TX_ZCD(I2S_T *i2s, uint32_t u32ChMask)
{
    if(u32ChMask == 0)
        i2s->CON &= ~(1ul << 16);
    else
        i2s->CON &= ~(1ul << 17);
}






 







 







 







 







 







 







 







 







 







 







 







 










 
static __inline void I2S_SET_MONO_RX_CHANNEL(I2S_T *i2s, uint32_t u32Ch)
{
    u32Ch == (1ul << 23) ?
    (i2s->CON |= (1ul << 23)) :
    (i2s->CON &= ~(1ul << 23));
}







 







 








 








 







 







 



 
uint32_t I2S_Open(I2S_T *i2s, uint32_t u32MasterSlave, uint32_t u32SampleRate, uint32_t u32WordWidth, uint32_t u32Channels, uint32_t u32DataFormat);
void I2S_Close(I2S_T *i2s);
void I2S_EnableInt(I2S_T *i2s, uint32_t u32Mask);
void I2S_DisableInt(I2S_T *i2s, uint32_t u32Mask);
uint32_t I2S_EnableMCLK(I2S_T *i2s, uint32_t u32BusClock);
void I2S_DisableMCLK(I2S_T *i2s);
void I2S_SetFIFO(I2S_T *i2s, uint32_t u32TxThreshold, uint32_t u32RxThreshold);

   


   

   



 

#line 12405 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\usbd.h"
 









 







 



 



 


typedef struct s_usbd_info
{
    const uint8_t *gu8DevDesc;             
    const uint8_t *gu8ConfigDesc;          
    const uint8_t **gu8StringDesc;         
    const uint8_t **gu8HidReportDesc;      
    const uint32_t *gu32HidReportSize;     
    const uint32_t *gu32ConfigHidDescIdx;  

} S_USBD_INFO_T;

extern const S_USBD_INFO_T gsInfo;

   





 







#line 65 "..\\Library\\StdDriver\\inc\\usbd.h"


 




 
#line 84 "..\\Library\\StdDriver\\inc\\usbd.h"

 
#line 93 "..\\Library\\StdDriver\\inc\\usbd.h"

 



 
#line 105 "..\\Library\\StdDriver\\inc\\usbd.h"

 







 



 
 
 














#line 148 "..\\Library\\StdDriver\\inc\\usbd.h"















   





 










 












 












 











 











 











 











 











 











 











 














 











 














 











 















 












 











 












 












 













 











 













 













 











 










 










 












 















 
static __inline void USBD_MemCopy(uint8_t *dest, uint8_t *src, int32_t size)
{
    while(size--) *dest++ = *src++;
}











 
static __inline void USBD_SetStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    int i;

    for(i = 0; i < 8; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xf) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0].CFGP;  
            u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

            *((volatile uint32_t *)(u32CfgAddr)) = (u32Cfg | (1ul << 1));
            break;
        }
    }
}









 
static __inline void USBD_ClearStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    int i;

    for(i = 0; i < 8; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xf) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0].CFGP;  
            u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

            *((volatile uint32_t *)(u32CfgAddr)) = (u32Cfg & ~(1ul << 1));
            break;
        }
    }
}











 
static __inline uint32_t USBD_GetStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    int i;

    for(i = 0; i < 8; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xf) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0].CFGP;  
            break;
        }
    }

    return ((*((volatile uint32_t *)(u32CfgAddr))) & (1ul << 1));
}


extern volatile uint8_t g_usbd_RemoteWakeupEn;
typedef void (*VENDOR_REQ)(void);            
typedef void (*CLASS_REQ)(void);             
typedef void (*SET_INTERFACE_REQ)(void);     
typedef void (*SET_CONFIG_CB)(void);        

 
void USBD_Open(const S_USBD_INFO_T *param, CLASS_REQ pfnClassReq, SET_INTERFACE_REQ pfnSetInterface);
void USBD_Start(void);
void USBD_GetSetupPacket(uint8_t *buf);
void USBD_ProcessSetupPacket(void);
void USBD_StandardRequest(void);
void USBD_PrepareCtrlIn(uint8_t *pu8Buf, uint32_t u32Size);
void USBD_CtrlIn(void);
void USBD_PrepareCtrlOut(uint8_t *pu8Buf, uint32_t u32Size);
void USBD_CtrlOut(void);
void USBD_SwReset(void);
void USBD_SetVendorRequest(VENDOR_REQ pfnVendorReq);
void USBD_SetConfigCallback(SET_CONFIG_CB pfnSetConfigCallback);
void USBD_LockEpStall(uint32_t u32EpBitmap);

   

   

   




 
#line 12406 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\pdma.h"
 









 



#line 16 "..\\Library\\StdDriver\\inc\\pdma.h"




 



 



 

 
 
 




 
 
 





 
 
 
#line 64 "..\\Library\\StdDriver\\inc\\pdma.h"


   



 









 










 











 











 











 











 












 
#line 161 "..\\Library\\StdDriver\\inc\\pdma.h"









 


void PDMA_Open(uint32_t u32Mask);
void PDMA_Close(void);
void PDMA_SetTransferCnt(uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount);
void PDMA_SetTransferAddr(uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl);
void PDMA_SetTransferMode(uint32_t u32Ch, uint32_t u32Periphral, uint32_t u32ScatterEn, uint32_t u32DescAddr);
void PDMA_Trigger(uint32_t u32Ch);
void PDMA_EnableInt(uint32_t u32Ch, uint32_t u32Mask);
void PDMA_DisableInt(uint32_t u32Ch, uint32_t u32Mask);




 



 



 

#line 12407 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\sc.h"
 








 











 



 



 
#line 37 "..\\Library\\StdDriver\\inc\\sc.h"

#line 48 "..\\Library\\StdDriver\\inc\\sc.h"


   




 





















 






















 











 
#line 121 "..\\Library\\StdDriver\\inc\\sc.h"











 
#line 141 "..\\Library\\StdDriver\\inc\\sc.h"










 
#line 160 "..\\Library\\StdDriver\\inc\\sc.h"










 
#line 179 "..\\Library\\StdDriver\\inc\\sc.h"







 









 









 








 
static __inline void SC_SetTxRetry(SC_T *sc, uint32_t u32Count)
{
    while((sc)->CTL & (1ul << 30));
    if((u32Count) == 0)         
    {
        (sc)->CTL &= ~((7ul << 20) | (1ul << 23));
    }
    else
    {
        (sc)->CTL = ((sc)->CTL & ~(7ul << 20)) | (((u32Count) - 1) << 20) | (1ul << 23);
    }
}







 
static __inline void  SC_SetRxRetry(SC_T *sc, uint32_t u32Count)
{
    while((sc)->CTL & (1ul << 30));
    if((u32Count) == 0)         
    {
        (sc)->CTL &= ~((7ul << 16) | (1ul << 19));
    }
    else
    {
        (sc)->CTL = ((sc)->CTL & ~(7ul << 16)) | (((u32Count) - 1) << 16) | (1ul << 19);
    }
}


uint32_t SC_IsCardInserted(SC_T *sc);
void SC_ClearFIFO(SC_T *sc);
void SC_Close(SC_T *sc);
void SC_Open(SC_T *sc, uint32_t u32CardDet, uint32_t u32PWR);
void SC_ResetReader(SC_T *sc);
void SC_SetBlockGuardTime(SC_T *sc, uint32_t u32BGT);
void SC_SetCharGuardTime(SC_T *sc, uint32_t u32CGT);
void SC_StopAllTimer(SC_T *sc);
void SC_StartTimer(SC_T *sc, uint32_t u32TimerNum, uint32_t u32Mode, uint32_t u32ETUCount);
void SC_StopTimer(SC_T *sc, uint32_t u32TimerNum);


   

   

   







 
#line 12408 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\scuart.h"
 








 











 



 



 













   




 

 







 











 









 









 










 









 



 







 










 










 










 









 


 













 














 
















 













 












 













 


void SCUART_Close(SC_T* sc);
uint32_t SCUART_Open(SC_T* sc, uint32_t u32baudrate);
uint32_t SCUART_Read(SC_T* sc, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);
uint32_t SCUART_SetLineConfig(SC_T* sc, uint32_t u32Baudrate, uint32_t u32DataWidth, uint32_t u32Parity, uint32_t  u32StopBits);
void SCUART_SetTimeoutCnt(SC_T* sc, uint32_t u32TOC);
void SCUART_Write(SC_T* sc, uint8_t *pu8TxBuf, uint32_t u32WriteBytes);

   

   

   







 
#line 12409 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\ps2.h"
 








 



 
 
 
#line 18 "..\\Library\\StdDriver\\inc\\ps2.h"









 



 




 

 
 
 










 











 











 












 
static __inline void PS2_CLEAR_TX_FIFO(void)
{
    ((PS2_T *) ((( uint32_t)0x40100000) + 0x00000))->PS2CON |= (1ul << 8);
    ((PS2_T *) ((( uint32_t)0x40100000) + 0x00000))->PS2CON &= ~(1ul << 8);
}









 










 











 










 










 










 










 










 










 










 










 










 



 
 
 

void PS2_Open(void);
void PS2_Close(void);
uint8_t PS2_Read(void);
int32_t PS2_Write(uint32_t *pu32Buf, uint32_t u32ByteCount);
void PS2_EnableInt(uint32_t u32Mask);
void PS2_DisableInt(uint32_t u32Mask);


   

   

   







 

#line 12410 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\clk.h"
 









 










 



 



 








 
 
 






#line 54 "..\\Library\\StdDriver\\inc\\clk.h"


 
 
 









#line 76 "..\\Library\\StdDriver\\inc\\clk.h"

#line 101 "..\\Library\\StdDriver\\inc\\clk.h"





#line 116 "..\\Library\\StdDriver\\inc\\clk.h"

#line 127 "..\\Library\\StdDriver\\inc\\clk.h"

 
 
 










#line 161 "..\\Library\\StdDriver\\inc\\clk.h"

#line 172 "..\\Library\\StdDriver\\inc\\clk.h"

#line 183 "..\\Library\\StdDriver\\inc\\clk.h"

#line 194 "..\\Library\\StdDriver\\inc\\clk.h"

#line 205 "..\\Library\\StdDriver\\inc\\clk.h"








 
 
 
#line 228 "..\\Library\\StdDriver\\inc\\clk.h"


 
 
 






 
 
 





 
 
 










#line 271 "..\\Library\\StdDriver\\inc\\clk.h"

#line 278 "..\\Library\\StdDriver\\inc\\clk.h"


 
 
 

 

#line 296 "..\\Library\\StdDriver\\inc\\clk.h"

#line 305 "..\\Library\\StdDriver\\inc\\clk.h"


#line 409 "..\\Library\\StdDriver\\inc\\clk.h"


#line 431 "..\\Library\\StdDriver\\inc\\clk.h"


   




 







 
static __inline uint32_t CLK_GetPLLClockFreq(void)
{
    uint32_t u32PllFreq = 0, u32PllReg;
    uint32_t u32FIN, u32NF, u32NR, u32NO;
    uint8_t au8NoTbl[4] = {1, 2, 2, 4};

    u32PllReg = ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->PLLCON;

    if(u32PllReg & ((1ul << 16) | (1ul << 18)))
        return 0;            

    if(u32PllReg & 0x00080000UL)
        u32FIN = (22118400UL);     
    else
        u32FIN = (12000000UL);      

    if(u32PllReg & (1ul << 17))
        return u32FIN;       

     
    u32NO = au8NoTbl[((u32PllReg & (3ul << 14)) >> 14)];
    u32NF = ((u32PllReg & (0x1FFul << 0)) >> 0) + 2;
    u32NR = ((u32PllReg & (0x1Ful << 9)) >> 9) + 2;

     
    u32PllFreq = (((u32FIN >> 2) * u32NF) / (u32NR * u32NO) << 2);

    return u32PllFreq;
}








 
static __inline void CLK_SysTickDelay(uint32_t us)
{
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = us * CyclesPerUs;
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL  = (0x00);
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2) | (1UL << 0);

     
    while((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16)) == 0);

     
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0;
}








 

static __inline void CLK_SysTickLongDelay(uint32_t us)
{
    uint32_t delay;
        
     
    delay = 233016UL;

    do
    {
        if(us > delay)
        {
            us -= delay;
        }
        else
        {
            delay = us;
            us = 0UL;
        }        
        
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = delay * CyclesPerUs;
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL  = (0x0UL);
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2) | (1UL << 0);

         
        while((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16)) == 0UL);

         
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0UL;
    
    }while(us > 0UL);
    
}

void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_DisablePLL(void);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);


   

   

   







 
#line 12411 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\acmp.h"
 








 











 



 



 

 
 
 
#line 44 "..\\Library\\StdDriver\\inc\\acmp.h"

   




 







 








 











 








 








 








 








 









 








 








 








 








 



 
void ACMP_Open(ACMP_T *, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysteresisEn);
void ACMP_Close(ACMP_T *, uint32_t u32ChNum);

   

   

   







 
#line 12412 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"
#line 1 "..\\Library\\StdDriver\\inc\\ebi.h"
 









 











 



 



 
 
 
 



 
 
 



 
 
 
#line 53 "..\\Library\\StdDriver\\inc\\ebi.h"

#line 61 "..\\Library\\StdDriver\\inc\\ebi.h"

   




 









 











 










 











 










 











 


void EBI_Open(uint32_t u32Bank, uint32_t u32DataWidth, uint32_t u32TimingClass, uint32_t u32BusMode, uint32_t u32CSActiveLevel);
void EBI_Close(uint32_t u32Bank);
void EBI_SetBusTiming(uint32_t u32Bank, uint32_t u32TimingConfig, uint32_t u32MclkDiv);

   

   

   







 
#line 12413 "..\\Library\\Device\\Nuvoton\\NUC230_240\\Include\\NUC230_240.h"


   


#line 2 "..\\src\\can_drv.c"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 3 "..\\src\\can_drv.c"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 4 "..\\src\\can_drv.c"
#line 1 "..\\src\\common.h"
#line 4 "..\\src\\common.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





#line 34 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
#line 57 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
#line 82 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









#line 114 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

#line 5 "..\\src\\common.h"
#line 6 "..\\src\\common.h"
#line 1 "..\\src\\bsp.h"









#line 94 "..\\src\\bsp.h"





 




 


 





#line 123 "..\\src\\bsp.h"






 
 


#line 141 "..\\src\\bsp.h"

#line 152 "..\\src\\bsp.h"











#line 174 "..\\src\\bsp.h"




#line 7 "..\\src\\common.h"
#line 1 "..\\src\\list.h"



#line 5 "..\\src\\list.h"

typedef struct list{
	struct list *next;
}list_t;




extern void list_insert_tail(list_t **head, list_t *node);
extern int list_remove(list_t **head, list_t *node);

#line 8 "..\\src\\common.h"









 

 

 




enum {
	FIRMWARE_TYPE_BOOTLOADER = 0,
	FIRMWARE_TYPE_APP,
	FIRMWARE_TYPE_DATA,
	FIRMWARE_TYPE_FONT,
	FIRMWARE_TYPE_UNKNOW,
};

typedef enum {
	TMR_ONCE_MODE = 0,		 
	TMR_AUTO_MODE = 1		 
}TMR_MODE_E;

typedef enum {
    COM0 = 0,
    COM1,
    COM2,
} COM_ID_t;

struct ledBlinkCfg {
	volatile uint32_t *pin;  
	uint16_t on_level;  
	uint16_t last_stat;  
	uint16_t on_time;   
	uint16_t off_time;  
	uint16_t cycles;  
	uint16_t counter;  
};

typedef void (*TimerCallback_t)(int param);

typedef struct {
	uint8_t Inuse;               
	volatile uint8_t Mode;		 
	volatile uint8_t Flag;		 
	volatile uint32_t Count;	 
	volatile uint32_t PreLoad;	 
    TimerCallback_t callback;
    int callback_param;
}SOFT_TMR;

typedef struct _ring_buffer{
    uint8_t *buf;
    volatile uint16_t rptr;
    volatile uint16_t wptr;
    volatile uint8_t isfull;
    volatile uint8_t rxchksum;



    union {
        volatile uint16_t send_cnt;
        volatile uint16_t recv_cnt;
    }rw_cnt;
    uint32_t start_time;
    
} RingBuffer_t;

typedef struct {
	UART_T *uart;
	list_t *rxqueue;
	list_t *txqueue;
	uint8_t *rxbuf;
	uint8_t tmprxbuf[6];
	uint16_t rx_cnt;
	uint16_t cmd_len;
	uint8_t chksum;
	uint8_t dma_pending;
	uint8_t cmd_pending;
	uint32_t time_start;
}ComInfo_t;

extern ComInfo_t comInfo;

typedef struct GBTimeStamp {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
}GBTimeStamp_t;

typedef struct _MsgHandler{
    uint8_t cmd;
    int8_t stat;
    uint8_t retry;
    RingBuffer_t *cmd_buf;
    RingBuffer_t *resp_buf;
    void (*func)(void);
}MsgObject_t;

typedef struct _ComPair{
    COM_ID_t in_com_id;
    COM_ID_t out_com_id;
}ComPair_t;

typedef struct CanMsgQueue{
    STR_CANMSG_T msg;
    uint8_t inuse;
    struct CanMsgQueue *next;
}CanMsgQueue_t;

typedef struct _DriverCardInfo{
	uint8_t factory_data[32];
    char lic_id[18];
    uint8_t expire_year; 
    uint8_t expire_month; 
    uint8_t expire_day; 
    char cert_id[18];
    uint8_t reserved[56];
    uint8_t chksum;
}DirverCardInfo_t;

 
struct FwInfo {
	uint32_t magic;
	char version[16];
	uint32_t load_addr;
	uint16_t fw_type;
	uint16_t header_size;
	uint32_t data_size;
	uint32_t data_crc;
	uint32_t utc_stamp;
	uint32_t reserved[5];
	uint32_t header_crc;
};

 
struct sys_cfg{
    uint32_t magic; 
    uint32_t check_sum; 
    uint8_t backlight_bright; 
    uint8_t backlight_timeout; 
    uint8_t menu_timeout; 
    uint8_t volume; 
    uint32_t alarm_cfg; 
    uint8_t passwd[8];
};

extern struct sys_cfg g_sSysCfg;

void mdelay(uint32_t ms);

void udelay(uint32_t us);

static __inline void mdelay2(uint32_t ms)
{
	while(ms--) {
		udelay(999);
	}
}

void DisableInterrupt(void);

void EnableInterrupt(void);

uint8_t BitPosToNum(uint32_t bit);
GPIO_T *GpioToPort(uint32_t gpio);

void TimersInit(void);
void SoftTimerStop(uint8_t id);
int SoftTimerCheckTimeout(uint8_t id);
void SoftTimerReload(uint8_t id);
void SoftTimerStart(uint8_t id, uint32_t period, TMR_MODE_E mode, TimerCallback_t func, int param);
void SoftTimerFree(uint8_t *id);
uint8_t SoftTimerAlloc(void);
uint32_t GetRunTime(void);
uint32_t CompareRunTime(uint32_t _LastTime);

void TimestampToRtcTime(uint32_t time, S_RTC_TIME_DATA_T *rtctime);

int FlashWritePage(uint32_t addr, void *buffer);

int FlashRead(uint32_t addr, uint32_t size, void *buffer);

void DataFlashInit(uint32_t u32Base);

void SysCfgInit(void);

void SysSaveConfig(void *cfg);

uint8_t crc8(uint8_t *buf, uint32_t size);

uint32_t crc32(uint32_t crc, uint8_t *buf, uint32_t size);

uint8_t GB19056_CalcCheckSum(uint8_t *buf, uint16_t len);

void RTC_Init(void);
void RTC_DeInit(void);

#line 223 "..\\src\\common.h"

void BoardKeyInit(void);
void BoardKeyDeInit(void);

int GetKeyVal(void);

int GetLongPressedKey(void);

void BoardComInit(void);
void BoardComSuspend(void);
void BoardIoInit(void);

void UartProcess(void);

uint16_t FormatCmdRespBuffer(uint16_t hdr_type, uint8_t cmd, uint8_t *buf, uint8_t *data, uint16_t data_len);

static __inline uint32_t m_ntohl(uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
}

static __inline uint16_t m_ntohs(uint8_t *buf)
{
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static __inline void m_htonl(uint32_t dat, uint8_t *buf)
{
    buf[0] = dat >> 24;
    buf[1] = (dat >> 16) & 0x00FF;
    buf[2] = (dat >> 8) & 0x00FF;
    buf[3] = dat & 0x00FF;
}

static __inline void m_htons(uint16_t dat, uint8_t *buf)
{
    buf[0] = dat >> 8;
    buf[1] = dat & 0x00FF;
}




extern CanMsgQueue_t *pCanRecvQHead[2];
extern CanMsgQueue_t *pCanSendQHead[2];
void CAN_Init(uint8_t port, uint32_t baud);
void CAN_DeInit(void);
void CAN_NormalModeTx(int port);;
CanMsgQueue_t *CanAllocMsg(int port, CanMsgQueue_t **qHead);
void CanMsgQueueSortedInsert(CanMsgQueue_t **qHead, CanMsgQueue_t *pMsg);
void CanMsgRemove(CanMsgQueue_t **qHead, CanMsgQueue_t *pMsg);
uint8_t CAN_SetRecvId(uint8_t port, uint32_t can_id);
void CAN_TriggerQueueSend(uint8_t port);
void CAN_ClearAllRecvId(uint8_t port);
void CanForwardUartMsg(uint8_t port, uint8_t *msg, uint16_t len);

void SPI_Init(SPI_T *port, uint32_t mode, uint32_t clk, uint8_t data_width);
void SPI_DeInit(SPI_T *port);
void SPI_WriteBytes(SPI_T *spi, uint8_t *buffer, uint32_t len);
void SPI_ReadBytes(SPI_T *spi, uint8_t *buffer, uint32_t len);
uint8_t SPI_WriteReadByte(SPI_T *spi, uint8_t byte);
void SPI_WriteWords(SPI_T *spi, uint16_t *buffer, uint32_t len);
void SPI_ReadWords(SPI_T *spi, uint16_t *buffer, uint32_t len);
uint16_t SPI_WriteReadWord(SPI_T *spi, uint16_t word);


void ADC_SampleInit(void);
uint16_t ADC_GetSampleData(uint8_t ch);
uint32_t ADC_GetChanVolt(uint8_t ch);

int8_t CheckRequestResponse(void);
int SendCmdRequest(uint8_t cmd, uint8_t *data, uint16_t len, uint8_t retry, uint8_t **resp);
uint32_t BcdBytesToBinary(uint8_t *bcd, uint8_t len);
void BcdTimeToGBTimeStamp(uint8_t *bcd, GBTimeStamp_t *tim);
void GBTimeStampToBcdTime(GBTimeStamp_t *tim, uint8_t *bcd);
void GetRTCTime(GBTimeStamp_t *tim);
extern MsgObject_t msg_obj;

void WDT_Feed(void);

void BuzzerOn(uint32_t freq, uint32_t duty_cycle, uint32_t time_ms);
void BuzzerStepCfg(uint8_t index, uint32_t freq, uint32_t duty_cycle, uint32_t time_ms, uint8_t start);
void BuzzerSimpleCfg(uint32_t freq, uint32_t periodOn, uint32_t periodOff, uint32_t times);
void BuzzerForceOff(void);
void hi_hexdump(const void *src, size_t len, size_t width);

void StartPowerMonitor(void);
void PowerDownFunction(void);
void LedBlink(uint8_t ledId, uint16_t onTime, uint16_t offTime, uint16_t cycles, uint8_t lastStat);

void SystemResume(void);
void SystemSuspend(void);
void FlushUartFifo(void);
void WaitUartFifoEmpty(void);

uint8_t CountBits(uint32_t val);

void UartCmdDispatch(void);
void UartRespDispatch(void);
void FlushTxRxQueue(void);
void IoOutput(uint8_t id, uint8_t val);
uint32_t IoInput(uint8_t id);
void SetSpeedPulse(uint16_t freq);
void AnalogOutput(uint8_t channel, uint8_t val);

void SendStartTestNotify(void);
void SendEndTestNotify(uint8_t val);
void StartCanSendTask(void);
void StartSendVoltageInfoTask(void);
void StopSendVoltageInfoTask(void);

void mcb_init(uint32_t mem_start, uint32_t mem_end);
void     *malloc(unsigned int size);
void     free(void *mem);


void SC2SendRadarData(void);
void SendWakeupNotify(void);



#line 5 "..\\src\\can_drv.c"
#line 6 "..\\src\\can_drv.c"
#line 1 "..\\src\\gb_t19056_def.h"



#line 5 "..\\src\\gb_t19056_def.h"












#line 33 "..\\src\\gb_t19056_def.h"

 
#line 46 "..\\src\\gb_t19056_def.h"



 


#line 67 "..\\src\\gb_t19056_def.h"

#line 79 "..\\src\\gb_t19056_def.h"

 
#line 89 "..\\src\\gb_t19056_def.h"

#line 98 "..\\src\\gb_t19056_def.h"

 










 
#line 117 "..\\src\\gb_t19056_def.h"

 















#line 142 "..\\src\\gb_t19056_def.h"

#line 150 "..\\src\\gb_t19056_def.h"

typedef struct {
	uint8_t header1;
	uint8_t header2;
	uint8_t cmd;
	uint8_t len_h;
	uint8_t len_l;
	uint8_t res;
	uint8_t *payload;
	uint8_t checksum;
}GB19056_PROTO_t;




#line 7 "..\\src\\can_drv.c"



extern volatile uint8_t print_onoff;

CAN_T  *tCAN[2] = {((CAN_T *) ((( uint32_t)0x40100000) + 0x80000)), ((CAN_T *) ((( uint32_t)0x40100000) + 0x84000))};
uint8_t can_timer_id[2] = {0xff, 0xff};

static uint32_t canid_filters[2][16];

static void CAN_XferMsgToUart(int port);

void CAN_Init(uint8_t port, uint32_t baud)
{
	if (port > 1)
		return;

    printf("CAN_Init(%d, %d)\n", port, baud);
    CAN_Close(tCAN[port]);
    CAN_ClearAllRecvId(port);

    if (port == 0) {
    	if (baud != 0) {
			 
			CLK_EnableModuleClock(((((1) & 0x03) << 30)|(((24) & 0x1f) << 0) | (((0x0) & 0x03) << 28)|(((0x0) & 0x07) << 25)|(((0x0) & 0x1f) << 20)| (((0x0) & 0x03) << 18)|(((0x0) & 0xff) << 10)|(((0x0) & 0x1f) << 5)));
			 
			((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->GPD_MFP &= ~((1UL<<6) | (1UL<<7));
			((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->GPD_MFP |= (1UL<<6) | (1UL<<7);
			NVIC_SetPriority(CAN0_IRQn, (1 << 2) - 3);
			NVIC_EnableIRQ(CAN0_IRQn);
			CAN_Open(tCAN[port],  baud, 0);
			CAN_EnableInt(tCAN[port], (1ul << 1)|(1ul << 2));      
    	} else {
    		CLK_DisableModuleClock(((((1) & 0x03) << 30)|(((24) & 0x1f) << 0) | (((0x0) & 0x03) << 28)|(((0x0) & 0x07) << 25)|(((0x0) & 0x1f) << 20)| (((0x0) & 0x03) << 18)|(((0x0) & 0xff) << 10)|(((0x0) & 0x1f) << 5)));
    		NVIC_DisableIRQ(CAN0_IRQn);
    	}
    } else {
    	if (baud != 0) {
			 
			CLK_EnableModuleClock(((((1) & 0x03) << 30)|(((25) & 0x1f) << 0) | (((0x0) & 0x03) << 28)|(((0x0) & 0x07) << 25)|(((0x0) & 0x1f) << 20)| (((0x0) & 0x03) << 18)|(((0x0) & 0xff) << 10)|(((0x0) & 0x1f) << 5)));
			 
			((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->GPD_MFP &= ~((1UL<<14) | (1UL<<15));
			((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->GPD_MFP |= (1UL<<14) | (1UL<<15);
			((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->ALT_MFP2 |= (1UL<<0) | (1UL<<0);
			NVIC_SetPriority(CAN1_IRQn, (1 << 2) - 2);
			NVIC_EnableIRQ(CAN1_IRQn);
			CAN_Open(tCAN[port],  baud, 0);
			CAN_EnableInt(tCAN[port], (1ul << 1)|(1ul << 2));
    	} else {
			CLK_DisableModuleClock(((((1) & 0x03) << 30)|(((25) & 0x1f) << 0) | (((0x0) & 0x03) << 28)|(((0x0) & 0x07) << 25)|(((0x0) & 0x1f) << 20)| (((0x0) & 0x03) << 18)|(((0x0) & 0xff) << 10)|(((0x0) & 0x1f) << 5)));
			NVIC_DisableIRQ(CAN1_IRQn);
		}
    }
}

void CAN_DeInit(void)
{






	 
	CLK_DisableModuleClock(((((1) & 0x03) << 30)|(((24) & 0x1f) << 0) | (((0x0) & 0x03) << 28)|(((0x0) & 0x07) << 25)|(((0x0) & 0x1f) << 20)| (((0x0) & 0x03) << 18)|(((0x0) & 0xff) << 10)|(((0x0) & 0x1f) << 5)) | ((((1) & 0x03) << 30)|(((25) & 0x1f) << 0) | (((0x0) & 0x03) << 28)|(((0x0) & 0x07) << 25)|(((0x0) & 0x1f) << 20)| (((0x0) & 0x03) << 18)|(((0x0) & 0xff) << 10)|(((0x0) & 0x1f) << 5)));
	 
	((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->GPD_MFP &= ~((1UL<<6) | (1UL<<7) | (1UL<<14) | (1UL<<15));
	((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->ALT_MFP &= ~(((1UL<<11) | (1UL<<28)) | ((1UL<<11) | (1UL<<28)));
	((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->ALT_MFP2 &= ~((1UL<<0) | (1UL<<0));
	NVIC_DisableIRQ(CAN0_IRQn);
	NVIC_DisableIRQ(CAN1_IRQn);

	CAN_Close(tCAN[0]);
	CAN_ClearAllRecvId(0);

	CAN_Close(tCAN[1]);
	CAN_ClearAllRecvId(1);
}

uint8_t FindFilterId(uint8_t port, uint32_t id)
{
	uint8_t i;
	for (i = 0; i < 16; i++) {
		if (canid_filters[port][i] == id)
			break;
	}
	return i;
}


 
CanMsgQueue_t CanRecvMsgQ[2][16] = {0};
CanMsgQueue_t *pCanRecvQHead[2] = {0};
CanMsgQueue_t CanSendMsgQ[2][16] = {0};
CanMsgQueue_t *pCanSendQHead[2] = {0};

CanMsgQueue_t *CanAllocMsg(int port, CanMsgQueue_t **qHead)
{
    CanMsgQueue_t *ret = 0;
    uint8_t i;
    DisableInterrupt();
    if (qHead == &pCanRecvQHead[port]) {
        if (*qHead == 0) {
            *qHead = CanRecvMsgQ[port];
            (*qHead)->inuse = 1;
            (*qHead)->next = 0;
            ret = *qHead;
        } else {
            for (i=0; i<16; i++) {
                if(!CanRecvMsgQ[port][i].inuse)
                    break;
            }

            if (i < 16) {
                CanRecvMsgQ[port][i].inuse = 1;
                CanRecvMsgQ[port][i].next = 0;
                ret = &CanRecvMsgQ[port][i];
            } else {
                ret = 0;
            }
        }
    } else if (qHead == &pCanSendQHead[port]) {
        if (*qHead == 0) {
            *qHead = CanSendMsgQ[port];
            (*qHead)->inuse = 1;
            (*qHead)->next = 0;
            ret = *qHead;
        } else {
            for (i=0; i<16; i++) {
                if(!CanSendMsgQ[port][i].inuse)
                    break;
            }
            if (i < 16) {
                CanSendMsgQ[port][i].inuse = 1;
                CanSendMsgQ[port][i].next = 0;
                ret = &CanSendMsgQ[port][i];
            } else {
                ret = 0;
            }
        }
    }
    EnableInterrupt();
    return ret;
}

void CanFreeMsg(CanMsgQueue_t *pMsg)
{
    if (pMsg) {
    	DisableInterrupt();
        pMsg->inuse = 0;
        pMsg->next = 0;
        EnableInterrupt();
    }
}

 
void CanMsgQueueSortedInsert(CanMsgQueue_t **qHead, CanMsgQueue_t *pMsg)
{
    CanMsgQueue_t *p = *qHead, *q;
    if (!qHead || !pMsg) {
    	return;
    }
    if(*qHead == 0) {
        *qHead = pMsg;
        return;
    } else if (*qHead == pMsg){
        return;
    } else {
    	DisableInterrupt();
        q = p;
        while(p) {
            if (p->msg.Id <= pMsg->msg.Id) {
                q = p;
                p = p->next;
            } else {
                if (p == *qHead) {
                    *qHead = pMsg;
                    pMsg->next = p;
                } else {
                    q->next = pMsg;
                    pMsg->next = p;
                }
                EnableInterrupt();
                return;
            }
        }
        q->next = pMsg;
        EnableInterrupt();
    }
}

void CanMsgRemove(CanMsgQueue_t **qHead, CanMsgQueue_t *pMsg)
{
    CanMsgQueue_t *p = *qHead, *q;
    if(*qHead == 0 || pMsg == 0) {
        return;
    } else {
        if (*qHead == pMsg) {
        	DisableInterrupt();
            *qHead = pMsg->next;
            pMsg->inuse = 0; 
            EnableInterrupt();
        } else {
        	DisableInterrupt();
            q = p;
            while(p != pMsg && p != 0) {q = p; p = p->next;}
            if (p) {
                q->next = pMsg->next;
                pMsg->inuse = 0;
            }
            EnableInterrupt();
        }
    }
}

void CanFreeMsgQueue(int port, CanMsgQueue_t **qHead)
{
    uint8_t i;
    DisableInterrupt();
    for(i=0;i<16;i++)
    {
        CanSendMsgQ[port][i].inuse = 0;
        CanSendMsgQ[port][i].next = 0;
    }
    *qHead = 0;
    EnableInterrupt();
}

uint8_t CAN_GetFreeRecvMsgBufCnt(int port)
{
    uint8_t i,ret = 0;
    for(i=0; i<16; i++) {
        if (!CanRecvMsgQ[port][i].inuse) {
            ret++;
        }
    }
    return ret;
}


static void CAN_XferMsgToUart(int port)
{
    uint16_t i=0;
    CanMsgQueue_t *pMsg, *pTmp;
	list_t *ptxlist_node;
	static uint16_t frame_len = 0;
	static uint8_t *pbuf;
	static uint8_t partial_flag = 0;
	uint8_t new_frame = 0;

    if (pCanRecvQHead[port] == 0) {
        return;
    }
	pMsg = pCanRecvQHead[port];
	while(pMsg) {
        if(pMsg->msg.Data[0] & 0x80) {
			if (partial_flag == 0 && (pMsg->msg.Data[0] & 0x7f) == 0) {
				partial_flag = 1;
				frame_len = ((uint16_t)pMsg->msg.Data[4] << 8) | pMsg->msg.Data[5];
				if (frame_len == 0) {
					partial_flag = 0;
					goto lable1;
				}
				ptxlist_node = malloc(sizeof(list_t) + frame_len);
				if (!ptxlist_node) {
					printf("uart tx queue is full!\n");
					new_frame = 0;
					partial_flag = 0;
					goto lable1;
				}
				pbuf = ((uint8_t *)(ptxlist_node) + sizeof(list_t));
			}
#line 285 "..\\src\\can_drv.c"
		} else {
			if (partial_flag == 1) {
				new_frame = 1;
			} else if((pMsg->msg.Data[0] & 0x7f) == 0){
				frame_len = ((uint16_t)pMsg->msg.Data[4] << 8) | pMsg->msg.Data[5];
				if (frame_len == 0) {
					partial_flag = 0;
					goto lable1;
				}
				ptxlist_node = malloc(sizeof(list_t) + i);
				if (!ptxlist_node) {
					printf("uart tx queue is full!\n");
					new_frame = 0;
					partial_flag = 0;
					goto lable1;
				}
				pbuf = ((uint8_t *)(ptxlist_node) + sizeof(list_t));
				new_frame = 1;
			}
			partial_flag = 0;
		}
		if (partial_flag || new_frame) {
			i = (uint16_t)(pMsg->msg.Data[0]&0x7F)*7;
			if (i > 256) {
				 
				printf("Msg length too long!");
				partial_flag = 0;
			} else {
				if (i + 7 > frame_len) {
					memcpy(&pbuf[i], &pMsg->msg.Data[1], frame_len - i);
				} else {
					memcpy(&pbuf[i], &pMsg->msg.Data[1], 7);
				}
			}
		}
lable1:
        pTmp = pMsg->next;
        CanMsgRemove(&pCanRecvQHead[port], pMsg);
        pMsg = pTmp;
        if (new_frame == 1) {
			ptxlist_node = ((list_t *)((uint8_t *)pbuf - sizeof(list_t)));
			list_insert_tail(&comInfo.txqueue, ptxlist_node);
			new_frame = 0;
			partial_flag = 0;
		}
    }
}

static void ForwardMsgToUart(STR_CANMSG_T *msg)
{
    uint16_t i=0;
	list_t *ptxlist_node;
	static uint16_t frame_len = 0;
	static uint8_t *pbuf;
	static uint8_t partial_flag = 0;
	uint8_t new_frame = 0;

	if(msg->Data[0] & 0x80) {
		if (partial_flag == 0 && (msg->Data[0] & 0x7f) == 0) {
			partial_flag = 1;
			frame_len = ((uint16_t)msg->Data[4] << 8) | msg->Data[5];
			if (frame_len == 0) {
				new_frame = 0;
				partial_flag = 0;
				return;
			}
			ptxlist_node = malloc(sizeof(list_t) + frame_len);
			if (!ptxlist_node) {
				printf("uart tx queue is full!\n");
				new_frame = 0;
				partial_flag = 0;
				return;
			}
			pbuf = ((uint8_t *)(ptxlist_node) + sizeof(list_t));
		}
#line 366 "..\\src\\can_drv.c"
	} else {
		if (partial_flag == 1) {
			new_frame = 1;
		} else if((msg->Data[0] & 0x7f) == 0){
			frame_len = ((uint16_t)msg->Data[4] << 8) | msg->Data[5];
			if (frame_len == 0) {
				new_frame = 0;
				partial_flag = 0;
				return;
			}
			ptxlist_node = malloc(sizeof(list_t) + frame_len);
			if (!ptxlist_node) {
				printf("uart tx queue is full!\n");
				new_frame = 0;
				partial_flag = 0;
				return;
			}
			pbuf = ((uint8_t *)(ptxlist_node) + sizeof(list_t));
			new_frame = 1;
		}
		partial_flag = 0;
	}
	if (partial_flag || new_frame) {
		i = (uint16_t)(msg->Data[0]&0x7F)*7;
		if (i > 256 || i >= frame_len) {
			 
			printf("Msg length too long!");
			partial_flag = 0;
			new_frame = 0;
		} else if (pbuf) {
			if (i + 7 > frame_len) {
				memcpy(&pbuf[i], &msg->Data[1], frame_len - i);
			} else {
				memcpy(&pbuf[i], &msg->Data[1], 7);
			}
		}
	}
	if (new_frame == 1 && pbuf) {
		ptxlist_node = ((list_t *)((uint8_t *)pbuf - sizeof(list_t)));
		list_insert_tail(&comInfo.txqueue, ptxlist_node);
		new_frame = 0;
		partial_flag = 0;
		pbuf = 0;
	}
}


 
 
 
void CAN_MsgInterrupt(CAN_T *tCan, uint32_t u32IIDR)
{
    uint8_t i;



	STR_CANMSG_T msg;

    u32IIDR -= 1;
    int port = (tCan == ((CAN_T *) ((( uint32_t)0x40100000) + 0x80000))) ? 0:1;

    if (tCan->STATUS & (1ul << 4)) {
        tCan->STATUS &= ~(1ul << 4);
#line 471 "..\\src\\can_drv.c"
		CAN_Receive(tCan, u32IIDR, &msg);
		ForwardMsgToUart(&msg);

    }
    if(tCan->STATUS & (1ul << 3)) {
        tCan->STATUS &= ~(1ul << 3);
        
        CanMsgRemove(&pCanSendQHead[port], pCanSendQHead[port]);
        CAN_NormalModeTx(port);
    }
}


 
 
 
void CAN0_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = ((CAN_T *) ((( uint32_t)0x40100000) + 0x80000))->IIDR;

    if(u8IIDRstatus == 0x00008000) {       
         
         
         
#line 509 "..\\src\\can_drv.c"
         
         
         
        if(((CAN_T *) ((( uint32_t)0x40100000) + 0x80000))->STATUS & (1ul << 7)) {
            printf("CAN%d BOFF INT(0x%02x)\n", 0, ((CAN_T *) ((( uint32_t)0x40100000) + 0x80000))->STATUS);
            


 
            ((CAN_T *) ((( uint32_t)0x40100000) + 0x80000))->CON &= (~((1ul << 0) | (1ul << 6)));
            CanFreeMsgQueue(0, &pCanSendQHead[0]);
            return;
        }

        if((((CAN_T *) ((( uint32_t)0x40100000) + 0x80000))->STATUS & (1ul << 6)) && !(((CAN_T *) ((( uint32_t)0x40100000) + 0x80000))->STATUS & ((1ul << 3)|(1ul << 4)))) {
            printf("CAN%d EWARN INT(0x%02x)\n", 0, ((CAN_T *) ((( uint32_t)0x40100000) + 0x80000))->STATUS) ;
             
            CanMsgRemove(&pCanSendQHead[0], pCanSendQHead[0]);
             
            if (pCanSendQHead[0]) {
                CAN_NormalModeTx(0);
            } else {
                ((CAN_T *) ((( uint32_t)0x40100000) + 0x80000))->CON &= ~(1ul << 2);
            }
        }
    } else if(u8IIDRstatus != 0) {
        CAN_MsgInterrupt(((CAN_T *) ((( uint32_t)0x40100000) + 0x80000)), u8IIDRstatus);

        CAN_CLR_INT_PENDING_BIT(((CAN_T *) ((( uint32_t)0x40100000) + 0x80000)), ((((CAN_T *) ((( uint32_t)0x40100000) + 0x80000))->IIDR) - 1));  
    } else if(((CAN_T *) ((( uint32_t)0x40100000) + 0x80000))->WU_STATUS == 1) {
        printf("Wake up\n");

        ((CAN_T *) ((( uint32_t)0x40100000) + 0x80000))->WU_STATUS = 0;     
    }

}

 
 
 
void CAN1_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = ((CAN_T *) ((( uint32_t)0x40100000) + 0x84000))->IIDR;

    if(u8IIDRstatus == 0x00008000) {       
         
         
         
#line 571 "..\\src\\can_drv.c"
         
         
         
        if(((CAN_T *) ((( uint32_t)0x40100000) + 0x84000))->STATUS & (1ul << 7)) {
            printf("CAN%d BOFF INT(0x%02x)\n", 1, ((CAN_T *) ((( uint32_t)0x40100000) + 0x84000))->STATUS);
            


 
            ((CAN_T *) ((( uint32_t)0x40100000) + 0x84000))->CON &= (~((1ul << 0) | (1ul << 6)));
            CanFreeMsgQueue(1, &pCanSendQHead[1]);
            return;
        }

        if((((CAN_T *) ((( uint32_t)0x40100000) + 0x84000))->STATUS & (1ul << 6)) && !(((CAN_T *) ((( uint32_t)0x40100000) + 0x84000))->STATUS & ((1ul << 3)|(1ul << 4)))) {
            printf("CAN%d EWARN INT(0x%02x)\n", 1, ((CAN_T *) ((( uint32_t)0x40100000) + 0x84000))->STATUS) ;
             
            CanMsgRemove(&pCanSendQHead[1], pCanSendQHead[1]);
             
            if (pCanSendQHead[1]) {
                CAN_NormalModeTx(1);
            } else {
                ((CAN_T *) ((( uint32_t)0x40100000) + 0x84000))->CON &= ~(1ul << 2);
            }
        }
    } else if(u8IIDRstatus != 0) {
        CAN_MsgInterrupt(((CAN_T *) ((( uint32_t)0x40100000) + 0x84000)), u8IIDRstatus);

        CAN_CLR_INT_PENDING_BIT(((CAN_T *) ((( uint32_t)0x40100000) + 0x84000)), ((((CAN_T *) ((( uint32_t)0x40100000) + 0x84000))->IIDR) - 1));  
    } else if(((CAN_T *) ((( uint32_t)0x40100000) + 0x84000))->WU_STATUS == 1) {
        printf("Wake up\n");

        ((CAN_T *) ((( uint32_t)0x40100000) + 0x84000))->WU_STATUS = 0;     
    }

}


 
 
 
void CAN_ResetIF(CAN_T *tCan, uint8_t u8IF_Num)
{
    if(u8IF_Num > 1)
        return;
    tCan->IF[u8IF_Num].CREQ     = 0x0;          
    tCan->IF[u8IF_Num].CMASK    = 0x0;
    tCan->IF[u8IF_Num].MASK1    = 0x0;          
    tCan->IF[u8IF_Num].MASK2    = 0x0;          
    tCan->IF[u8IF_Num].ARB1     = 0x0;          
    tCan->IF[u8IF_Num].ARB2     = 0x0;          
    tCan->IF[u8IF_Num].MCON     = 0x0;          
    tCan->IF[u8IF_Num].DAT_A1   = 0x0;          
    tCan->IF[u8IF_Num].DAT_A2   = 0x0;          
    tCan->IF[u8IF_Num].DAT_B1   = 0x0;          
    tCan->IF[u8IF_Num].DAT_B2   = 0x0;          
}

 
 
 

void CAN_NormalModeTx(int port)
{
    uint32_t u32MsgNum;
    CAN_T *tCan = (port == 0) ? ((CAN_T *) ((( uint32_t)0x40100000) + 0x80000)):((CAN_T *) ((( uint32_t)0x40100000) + 0x84000));

    if(tCan->STATUS & (1ul << 7)) {
        printf("BOFF(0x%02x)\n", tCan->STATUS);
        tCan->CON &= (~((1ul << 0) | (1ul << 6)));
        return;
    }

    if (pCanSendQHead[port] && !(tCan->IF[1].CREQ & (1ul << 15))) {
        u32MsgNum = ((uint32_t)pCanSendQHead[port] - (uint32_t)CanSendMsgQ[port])/sizeof(CanMsgQueue_t) + 16;
        CAN_SetTxMsg(tCan, (u32MsgNum), &pCanSendQHead[port]->msg);
         
        tCan->CON |= (1ul << 2);
        CAN_TriggerTxMsg(tCan, (u32MsgNum));
    }
}

 
 
 
uint8_t CAN_SetRecvId(uint8_t port, uint32_t can_id)
{
	uint8_t index;

	if (FindFilterId(port, can_id) < 16) {
		 
		return 0;
	}
	index = FindFilterId(port, 0xFFFFFFFF);
	if (index >= 16) {
		 
		return 1;
	}

	printf("CAN_SetRecvId(port=%d, index=%d, id=0x%x)\n", port, index, can_id);

    if(CAN_SetRxMsg(tCAN[port], (index), can_id>>31, can_id&0x7FFFFFFF) == 0) {
        printf("Set Rx Msg Object failed\n");
        return 1;
    }
    canid_filters[port][index] = can_id;
    return 0;
}

void CAN_ClearAllRecvId(uint8_t port)
{
	uint32_t i;

	for (i=0; i<16; i++) {
		tCAN[port]->IF[1].ARB2 = 0;
		canid_filters[port][i] = 0xFFFFFFFF;
	}
}

void CAN_TriggerQueueSend(uint8_t port)
{
    CAN_NormalModeTx(port);
}

void CanForwardUartMsg(uint8_t port, uint8_t *msg, uint16_t len)
{
	uint8_t *p = msg;
	CanMsgQueue_t *pCanMsg;
	uint16_t i=0, part_flag, tmp;
	uint8_t pid = 0;
	
	do {
		pCanMsg = CanAllocMsg(port, &pCanSendQHead[port]);
		if (pCanMsg) {
			pCanMsg->msg.Id = 0x1FF5A000;
			pCanMsg->msg.FrameType = 1;
			pCanMsg->msg.IdType = 1;
			pCanMsg->msg.DLC = 8;
			tmp = len - i;
			if (len <= 7) {
				part_flag = 0;
				pCanMsg->msg.DLC = len + 1;
			} else if (tmp > 7) {
				part_flag = 0x80;
			} else {
				part_flag = 0;
				if (tmp <= 7) {
					pCanMsg->msg.DLC = tmp + 1;
				}
			}

			pCanMsg->msg.Data[0] = part_flag | pid;
			memcpy(&pCanMsg->msg.Data[1], p+i, 7);

			CanMsgQueueSortedInsert(&pCanSendQHead[port], pCanMsg);
			i += 7;
			pid += 1;
		} else {
			printf("CAN send queue is full, drop %d packets\n", len-i);
			break;
		}
	} while(i < len);
	if (len) {
		CAN_TriggerQueueSend(port);
	}
}

