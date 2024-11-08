#include "nu32dip.h"
#include "ina219.h"
#include "utilities.h"
#include "encoder.h"
#include <stdlib.h>


#define _PR2 9599
#define _PR3 2399
#define _PR4 14999
#define PLOTSPT 100
#define TRACKPTS 100



static volatile int duty_cycle;
static volatile int count = 0;
static volatile int h_count = 0;
static volatile int traj_count = 0;
static volatile int data_kept = 0;
static volatile int CURarray[PLOTSPT];

static volatile float Kp = 0.2;
static volatile float Ki = 0.2;
static volatile float eint = 0;
static volatile float pos_eint = 0;
static volatile float eprev_pos = 0;
static volatile float RefArray[PLOTSPT]
static volatile float _2RefArray[TRACKPTS]
static volatile float TrajRef[TRACKPTS]

static volatile float pKp=0.9
static volatile float  pKi=0.0
static volatile float   pKd=0.3
static volatile float  usr_angle=0.0
static volatile float commanded_current = 16.0
static volatile float  POSarray[PLOTPTS]
static volatile float  _2POSarray[TRACKPTS];


void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Current_Controller(void)
{
    swtich(get_mode()){
      case 0:{
        //IDLE
        LATBits.LATB2 = 0;
        0C1RS = 0;
        break;
      }
      case 1:{
        //PWMN CASE 
        LATBits.LATB2 = 1;
        0C1RS = (int) ((float)duty_cycle/-100.0 * (_PR3 + 1));  
        break;
      }
      case 1:{
        //PWMN CASE 
        LATBits.LATB2 = 1;
        0C1RS = (int) ((float)duty_cycle/-100.0 * (_PR3 + 1));
        
        break;
      }
      case 2:{
        //ITest case
        int update = 0;
        if(count < 25){
          update = 200
        }
        else if(count<50){
          update = -200;
        }
        else if(count<75){
          update = 200;
        }
        else{ 
          update = -200
        }

        int c_senor_val = INA219_read_current();
        int off = -1*(update-c_senor_val);
        eint = eint + error
        float t_e = Kp*error + Ki*eint;
      }
      case 3:{//Hold
        int c_senor_val = INA219_read_current();
        float error = -1*(commanded_current - current_sensor);
        eint = eint + error;
        float t_e = Kp*error + Ki*eint;
        if (t_e > 100.0){
          t_e = 100;

        }
        else if(t_e < -100.0){
          t_e = -100.0;
          
        }

        if(t_e < 0){
          OC1RS = (int)((t_e/-100.0)*_PR3);
          LATbits.LABTB2 = 1;
          
        }
        else{
          OC1RS = (int)((t_e/-100.0)* _PR3);
          LATbits.LABTB2 = 0;
        }

        break;


      }
      case 4: {//TRACk

        int c_senor_val = INA219_read_current();
        float error = -1*(commanded_current - current_sensor);
        eint = eint + error;
        float t_e = Kp*error + Ki*eint;
        if (t_e > 100.0){
          t_e = 100;

        }
        else if(t_e < -100.0){
          t_e = -100.0;
          
        }

        if(t_e < 0){
          OC1RS = (int)((u/-100.0)*_PR3);
          LATbits.LABTB2 = 1;
          
        }
        else{
          OC1RS = (int)((u/-100.0)* _PR3);
          LATbits.LABTB2 = 0;
        }

        break;

      }
  
    }
     IFS0bits.T2IF = 0;       
}



void __ISR(_TIMER_4_VECTOR, IPL4SOFT) Position_Controller(void)
{
    swtich(get_mode()){

      case 3:{//Hold
      
        WriteUART2("a"); 
        while(!get_encoder_flag()){} 
        set_encoder_flag(0); 
        int potential = get_encoder_count();

        float pot_ang = (float) p * 360.0/1400.0;
        int error = usr_angle - pot_ang;
        pos_eint = pos_eint + error;
        float t_e = pKp_error + pKi*pos+ pKd*(error-eprev_pos)/0.02;
        commanded_current = t_e;
        eprev_pos = error;
        POSarray[count_h] = pot_ang;
        RefArray[count_h] = usr_angle;
        count_h++;

        if (count_h == PLOTPTS) {
          set_mode(IDLE);
          pos_eint =0;
          eprev_pos =0;
          count_h = 0;

          
      }
      break;


      }
      case 4: {//TRACk

        WriteUART2("a"); 
        while(!get_encoder_flag()){} 
        set_encoder_flag(0); 
        int potential = get_encoder_count();

        float pot_ang = (float) p * 360.0/1400.0;
        int error = usr_angle - pot_ang;
        pos_eint = pos_eint + error;
        float t_e = pKp_error + pKi*pos+ pKd*(error-eprev_pos)/0.02;
        commanded_current = t_e;
        eprev_pos = error;
        _2POSarray[traj_count] = pot_ang;
        _2RefArray[traj_count] = TrajRef[count_traj]
        traj_count++;

        if (count_h == PLOTPTS) {
          traj_count = 0;

          
      }

        break;


      }
  
    }
     IFS0bits.T2IF = 0;       
}

int main() 
{
  char strg[200];
  set_mod(IDLE);
  NU32DIP_Startup();
  UART2_Startup();
  INA219_Startup();
  TRISBCLR = 0x4;
  RPA0Rbits.RPA0R = 0b0101;

  IPC2bits.T2IP = 5;             
  IPC2bits.T2IS = 0;
  IFS0bits.T2IF = 0;             
  IEC0bits.T2IE = 1;            
  IPC4bits.T4IP = 5;                
  IPC4bits.T4IS = 1;
  IFS0bits.T4IF = 0;               
  IEC0bits.T4IE = 1;            


  T4CONbits.TCKPS = 4;
  T3CONbits.TCKPS = 0; 
  T2CONbits.TCKPS = 0;

  T2CONbits.ON = 1;
  T3CONbits.ON = 1;
  T4CONbits.ON = 1;

  PR2 = _PR2;
  PR3 = _PR3;
  PR4 = _PR4;

  TMR2 = 0;
  TMR3 = 0;
  TMR4 = 0;

  OC1CONbits.OCM = 0b110; 
  OC1CONbits.OCTSEL = 1;  
  OC1CONbits.ON = 1;     
  OC1RS = 0;
  OC1R = 0;

  while(1){
    switch(strg[0]){
      case 'f':
      {
        set_mode(PWM);
        int n = 0;
        NU32DIP_ReadUART1(strg,200);
        sscanf(strg, "%d", &n);
        sprintf(buffer,"%d\r\n", n);
        NU32DIP_WriteUART1(strg);

        duty_cycle = n;
        break;
      }
      
      case 'g':
      {
        float kpt = 0, kit = 0;
        NU32DIP_ReadUART1(strg,200);
        sscanf(strg, "%f %f", &kpt, &kit);
        Kp = kpt;
        Ki = kit;
        break;
      }
      case 'h':
      {
        sprintf(strg, "%f\r\n", Kp);
        NU32DIP_WriteUART1(strg);

        sprintf(strg, "%f\r\n", Ki);
        NU32DIP_WriteUART1(strg);
        break;
      }

      case 'k':
      {
        StoringData = 1;
        set_mode(ITEST);
        while (StoringData) {
          ;
        } 
        sprintf(strg, "%d\r\n", PLOTPTS);
        NU32DIP_WriteUART1(strg);

        for (int i=0; i<PLOTPTS; i++) {
         
          sprintf(strg, "%d %d \r\n", CURarray[i], REFarray[i]);
          NU32DIP_WriteUART1(strg);
        }

        break;
      }
      case 'm': 
      {
        for (int i=0; i < TRACKPTS; i++)
        {
          float refpos = 0.0;
          NU32DIP_ReadUART1(strg,BUF_SIZE);
          sscanf(strg, "%f\n", &refpos);

          RefTraj[i] = refpos;
        }

        break;
      }
      case 'n': 
      {
        for (int i=0; i < TRACKPTS; i++)
        {
          float refpos = 0.0;
          NU32DIP_ReadUART1(strg,200);
          sscanf(strg, "%f\n", &refpos);
          RefTraj[i] = refpos;
        }
        break;
      }
      case 'o':
      {
        sprintf(strg, "%d\r\n", TRACKPTS); 
        NU32DIP_WriteUART1(strg);

        set_mode(TRACK);

        for (int i=0; i<TRACKPTS; i++) { 
          sprintf(strg, "%f %f \r\n", POSarray2[i], REFarray2[i]);
          NU32DIP_WriteUART1(strg);
        }
        break;


    }
    case 'p':
      {
        set_mode(IDLE);
        break;
      }
  }



}
  return 0;
}