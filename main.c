/**
*    Desenvolvido por: Aldair Wontroba
*    Data do inicio de criação do codigo: 01/09/2018
*    Data da ultima atualização: 28/03/2020
*    Descrição: Software para processador TMS320F28377S
*               O codigo apresentado aqui tem a função de controlar o hardware
*               envolvido desde os perifericos do processador até a placa de
*               acquisição de dados.
*               Funções de softwre:
*                   Acquisição de dados, amostragem controlada com medição de
*                   frequencia;
*                       SPI, Mcbsp TIMERS e interrupções são usadas neste
*                       processo.
*                   comunicação serial e protocolo especial para captuda de
*                   dados e controle.
*                   Rologio interno e controle de processo do processador.
*               O codigo desenvolvido aki é baseado em interrupções, ou seja, em
*               nenhum momento fica preso em um laço infinito. O realmente fica
*               em desuso quando não está dentro  de uma interrupção.
*               Ao modificar este codigo aprenda a usar interrupções antes de
*               mais nada.
*    Contato: aldair.wontroba@hotmail.com
*/
#define metodo 2   //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<   METODO

// 0 torres
// 1 erica
// 2 Aldair
// 3 brhama
// 4 kavi    lixo - n�o vou implementar essa merda
// 5 Subranain
// 6 somente fft


#include "F28x_Project.h"
#include "F28377S_MyDefinition.h"
#include <math.h>
#include "complex.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define PI 3.14159265359

#define VA 0
#define VB 1
#define VC 2
#define IA 3
#define IB 4
#define IC 5
#define IN 6


#define Ia 0
#define Ib 1
#define Ic 2
#define In 3

#define Va 0
#define Vb 1
#define Vc 2


#define FFTLenWindow  1
#define ChannelsRead  9
#define feqRede 60
#define fase_REF 0
#define Amostragen_flag 1
#define imputmode 1 //0-SEL; 1-Normal
#define RANGE 10
#define ATP_NORMAL 0


#define faseemfai 6

#define my_fase_REF 6

/*******************************************************************************
 *              variaveis globais
 ******************************************************************************/
#if metodo==6
    #define HarmNum  4
    #define amostragem 128
    #define counter_process 128
    #define Nciclos 60
#endif
////////////////////////////////////////////////////////////////////////////////
#if metodo==0 //torres2014

#define HarmNum  10
#define amostragem 128
#define counter_process 32
#define Nciclos 30

    int16 torres_delay = 200;
    int16 torres_k = 1.05;
    int16 torres_first_detection[3];
    int16 torres_time_detection[3];
    int16 torres_FAI_detect[3];


    float torres_THD[3];
    float torres_oddharm[3];
    float torres_evenharm[3];

#endif
///////////////////////////////////////////////////////////////////////////////////////////////////
#if metodo==1

#define HarmNum  6
#define amostragem 128
#define counter_process 32
#define Nciclos 60


#define Er_fase_REF 0

    float Er_relation2[3];
    float Er_relation3[3];
    float Er_relation5[3];
    float pickup2 = 0.0001;
    float pickup3 = 0.002;
    float pickup5 = 0.001;

    int Er_nciclos = 5;
    int Er_divisor;

    Uint16 Er_harm_detected[3];
    Uint16 Er_fase_detected[3];
    Uint16 Er_harm_first_detection[3];
    Uint16 Er_first_fase_detection[3];
    Uint16 Er_FAI_detect[3];
    Uint16 Er_time_fase_detection[3];
    Uint16 Er_harm_time_detection[3];
    int Er_time=200;

    float Er_modulo1harm;
    float Er_angulo1harm;
    float Er_modulo3harm;
    float Er_angulo3harm;
    float Er_fftS_media[ChannelsRead][HarmNum];
    float Er_fase_media[ChannelsRead][HarmNum];


#endif
///////////////////////////////////////////////////////////////////////////////////////////////////
#if metodo==2

    #define HarmNum  8
    #define amostragem 64
    #define counter_process 32
    #define Nciclos 40

    int Alda_FAI_detect_counter;
    int Alda_firt_detection;
    int Alda_FAI_detect;
    int Alda_fase_detected;
    int Alda_harm_detected;
    float phaseIA[3];
    float phaseIB[3];
    float phaseIC[3];
    int Alda_classificador[3];
    int Alda_detection;
    int Alda_CBs = 0;
    int Alda_CBsf[3];
    float Alda_HarmPar;
    float Alda_HarmImpar;
    float Alda_sfftM[3];
    float Alda_faseM[3];
    int Alda_divisor;
    float Alda_I1m;
    float Alda_I2m;
    float Alda_I1mant;
    float Alda_I2mant;
    float Alda_der_I2m;

    float Alda_Ia_holdi;
    float Alda_fb2a_holdi;
    float Alda_fc2a_holdi;
    float Alda_Ib_holdi;
    float Alda_fa2b_holdi;
    float Alda_fc2b_holdi;
    float Alda_Ic_holdi;
    float Alda_fa2c_holdi;
    float Alda_fb2c_holdi;
    float Alda_Ia_holdf;
    float Alda_fb2a_holdf;
    float Alda_fc2a_holdf;
    float Alda_Ib_holdf;
    float Alda_fa2b_holdf;
    float Alda_fc2b_holdf;
    float Alda_Ic_holdf;
    float Alda_fa2c_holdf;
    float Alda_fb2c_holdf;

    float Alda_Ia_holdiant;
    float Alda_fb2a_holdiant;
    float Alda_fc2a_holdiant;
    float Alda_Ib_holdiant;
    float Alda_fa2b_holdiant;
    float Alda_fc2b_holdiant;
    float Alda_Ic_holdiant;
    float Alda_fa2c_holdiant;
    float Alda_fb2c_holdiant;

    float Alda_relation;

    float Alda_delta_Ia;
    float Alda_delta_Ib;
    float Alda_delta_Ic;

    float Alda_I1_holdi;
    float Alda_I1_holdf;
    float Alda_I2_holdi;
    float Alda_I2_holdf;

    int Alda_count_CBs;
    int Alda_flag_CBs;

    int Alda_reset_CB=0;
    int Alda_nciclos=5;

    int Alda_inicial_counter = 0;
    int Alda_inicial_set = 0;
    int Alda_inicial_freze = 1000;

    float Alda_I2mant2;
    float Alda_I1mant2;

    float Alda_I2mant1;
    float Alda_I1mant1;

    float   Alda_Ia_holdiant1;
    float Alda_fb2a_holdiant1;
    float Alda_fc2a_holdiant1;
    float   Alda_Ib_holdiant1;
    float Alda_fa2b_holdiant1;
    float Alda_fc2b_holdiant1;
    float   Alda_Ic_holdiant1;
    float Alda_fa2c_holdiant1;
    float Alda_fb2c_holdiant1;
    float   Alda_Ia_holdiant2;
    float Alda_fb2a_holdiant2;
    float Alda_fc2a_holdiant2;
    float   Alda_Ib_holdiant2;
    float Alda_fa2b_holdiant2;
    float Alda_fc2b_holdiant2;
    float   Alda_Ic_holdiant2;
    float Alda_fa2c_holdiant2;
    float Alda_fb2c_holdiant2;
    float   Alda_Ia_holdiant3;
    float Alda_fb2a_holdiant3;
    float Alda_fc2a_holdiant3;
    float   Alda_Ib_holdiant3;
    float Alda_fa2b_holdiant3;
    float Alda_fc2b_holdiant3;
    float   Alda_Ic_holdiant3;
    float Alda_fa2c_holdiant3;
    float Alda_fb2c_holdiant3;

    int Alda_divisor_fasor;
    int Alda_nciclos_fasor = 5;

    int Alda_CBsp = 0;

    float Alda_modulo1harm;
    float Alda_angulo1harm;
    float Alda_modulo3harm;
    float Alda_angulo3harm;
    float Alda_fftS_media[ChannelsRead][HarmNum];
    float Alda_fase_media[ChannelsRead][HarmNum];

#endif
///////////////////////////////////////////////////////////////////////////////////////////////////
#if metodo==3

#define HarmNum  2
#define amostragem 64
#define counter_process 1
#define Nciclos 120

#define MMenable 1
#define MMSELEN 2
#define CODOLEN 1
#define Br_atraso 8

#define Br_len 3
#define Br_len_C amostragem

    int16 Br_time[3] = {0,0,0};
    int16 Br_inicialset[3] = {0,0,0};
    int16 Br_FAI_detect[3] = {0,0,0};
    int16 Br_tw = 32;
    float Br_dilatacao[3][Br_len];
    float Br_erosao[3][Br_len];
    float Br_SE[MMSELEN];
    float Br_closed[3][Br_len];
    float Br_opened[3][Br_len];
    float CODO[3][Br_len_C];
    float Br_th[3] = {0.01, 0.01, 0.01};

    float prave;

#endif
///////////////////////////////////////////////////////////////////////////////////////////////////
#if metodo==4

#define HarmNum  2
#define amostragem 64
#define counter_process 64
#define Nciclos 60

#define Ka_mmf_len 5
#define Ka_dif_len 64
#define Ka_asf_len 24
#define Ka_mdf_len 128
#define Ka_atraso 32

    int Ka_flag_counter[3] = {0,0,0};
    int Ka_flag_L1 = 0;

    float Ka_A1[3];
    float Ka_A2[3];
    float Ka_B1[3];
    float Ka_B2[5];

    int Ka_mmf_pv = 0;

    float Ka_mmf_DI1[6][Ka_mmf_len];
    float Ka_mmf_DI2[6][Ka_mmf_len];
    float Ka_mmf_EI1[6][Ka_mmf_len];
    float Ka_mmf_EI2[6][Ka_mmf_len];
    float Ka_mmf_Av[6];

    int Ka_dif_pv = 0;

    float Ka_dif[6][Ka_dif_len];

    int Ka_asf_pv = 0;

    float Ka_asf_CO_1[6][Ka_asf_len];
    float Ka_asf_CO_2[6][Ka_asf_len];
    float Ka_asf_CO_3[6][Ka_asf_len];
    float Ka_asf_CO_4[6][Ka_asf_len];
    float Ka_asf_CO_5[6][Ka_asf_len];
    float Ka_asf_CO_6[6][Ka_asf_len];
    float Ka_asf_CO_7[6][Ka_asf_len];
    float Ka_asf_CO_8[6][Ka_asf_len];

    float Ka_asf_OC_1[6][Ka_asf_len];
    float Ka_asf_OC_2[6][Ka_asf_len];
    float Ka_asf_OC_3[6][Ka_asf_len];
    float Ka_asf_OC_4[6][Ka_asf_len];
    float Ka_asf_OC_5[6][Ka_asf_len];
    float Ka_asf_OC_6[6][Ka_asf_len];
    float Ka_asf_OC_7[6][Ka_asf_len];
    float Ka_asf_OC_8[6][Ka_asf_len];

    int Ka_mdf_pv = 0;

    float Ka_mdf[6][Ka_mdf_len];

    int Ka_counter;
    int Ka_inicio;
    float Ka_find_tall;
    int Ka_find_tall_index;
    float Ka_tall;
    int Ka_index;
    int Ka_flag_tall_rand;
    float Ka_Iinc;
    int Ka_flag_first_detection = 0;
    int Ka_fase = IA;
    int Ka_counter_16ms = 0;
    int Ka_counter_1s = 0;
    int Ka_second_detection = 0;;



#endif
///////////////////////////////////////////////////////////////////////////////////////////////////
#if metodo==5

    #define HarmNum  4
    #define amostragem 128
    #define counter_process 32
    #define Nciclos 60

    int Sub_FAI_detect_counter;
    int Sub_firt_detection;
    int Sub_FAI_detect;
    float Sub_relation;

    int Sub_nciclos = 10;
    int Sub_divisor;

    int16 Sub_time = 200;

    float Sub_modulo1harm;
    float Sub_angulo1harm;
    float Sub_modulo3harm;
    float Sub_angulo3harm;

    float Sub_fftS_media[ChannelsRead][HarmNum];
    float Sub_fase_media[ChannelsRead][HarmNum];

#endif
///////////////////////////////////////////////////////////////////////////////////////////////////

	int16 DATA[16];

	int16 DATA_VECTOR[ChannelsRead][FFTLenWindow*amostragem*Nciclos];
	int16 DATA_VECTOR_EX[ChannelsRead][FFTLenWindow*amostragem*Nciclos];

	float DATA_VECTOR_HOLD[ChannelsRead][FFTLenWindow*amostragem];

	float fftS[ChannelsRead][HarmNum];
	float fftA[ChannelsRead][HarmNum];
	float phase[ChannelsRead][HarmNum];

	float Cfc[HarmNum][FFTLenWindow*amostragem];
	float Cfs[HarmNum][FFTLenWindow*amostragem];



#if ATP_NORMAL
    const float ATP_ajuste[16] =
             {1.5,
              1.5,
              1.5,
              1.5,
              1.5,
              1.5,
              1.5,
              1.5,
              1.5,
              1.5,
              1.5,
              1.5,
              1.5,
              1.5,
              1.5,
              1.5};
#else
    const float ATP_ajuste[16] =
             {1.0,
              1.0,
              1.0,
              1.0,
              1.0,
              1.0,
              1.0,
              1.0,
              1.0,
              1.0,
              1.0,
              1.0,
              1.0,
              1.0,
              1.0,
              1.0};
#endif

#if RANGE==10
// bit to volt conversão
  float DATA_V_I_BASE[16] =
                            {0.000305,
                            0.000305,
                            0.000305,
                            0.000305,
                            0.000305,
                            0.000305,
                            0.000305,
                            0.000305,
                            0.000305,
                            0.000305,
                            0.000305,
                            0.000305,
                            0.000305,
                            0.000305,
                            0.000305,
                            0.000305};
#else
  float DATA_V_I_BASE[16] =
                            {0.000152,
                             0.000152,
                             0.000152,
                             0.000152,
                             0.000152,
                             0.000152,
                             0.000152,
                             0.000152,
                             0.000152,
                             0.000152,
                             0.000152,
                             0.000152,
                             0.000152,
                             0.000152,
                             0.000152,
                             0.000152};
#endif
// relação de transformação
  const int16 DATA_V_I_RTC_RTP[16] =  {1500,
                                 1500,
                                 1500,
                                 50,
                                 50,
                                 50,
                                 10,
                                 1,
                                 1,
                                 1,
                                 1,
                                 1,
                                 1,
                                 1,
                                 1,
                                 1};

  const int16 DATA_V_I_RTC_RTP_EX[16] =  {1,
                                          1,
                                          1,
                                          1,
                                          1,
                                          1,
                                 1,
                                 1,
                                 1,
                                 1,
                                 1,
                                 1,
                                 1,
                                 1,
                                 1,
                                 1};
 char palavra[][30] = {"start.adc",
                    "stop.adc",
                    "get.data",
                    "reset FAI",
                    "Event report"};

/////////////////////
float constant_1 = 0;
float constant_dois = 0;
float CPU_percent = 0;
float time_ref = 0;
Uint32 timercontrol = 0;
int32 count_zerocross_timer = 0;
float timersave = 0;
float timer_ajust = 0;
float time_cicle = 0;
float time_ruy = 0;
float time_ruy_anterior = 0;
float frequencia = 60;
float iniciotimer = 0;
float fimtimer = 0;

// Variaveis utilizadas para enviar dados via serial

char *msgS;   //Ponteiro utilizado para enviar STRINGS
char *msgR;   //Ponteiro utilizado para receber STRINGS


int32 send_length = 0; //Tamanho do vetor de dados a ser enviado no serial
int send_mode = 0;    //Modo 0 = STRINGS; 1 = DADOS
int send_data = 0;

int lenght_pv;

int fasefai = faseemfai;

int16 Event_report = 0;
int16 event_enable = 0;

int16 pv_tctp = 0;
int16 time_led = 0;
int16 ch_pv = 0;
int16 ch_pvR = 0;
int16 ch_pvS = 0;
int16 j = 0;
int16 jj = 0;
int16 k = 0;
int16 kk = 0;
int16 pv = 0;
int16 MMpv=0;
int16 CODOpv=0;
int16 point_pv = 0;
int16 pv_ant = 0;
int16 pv_hold = 0;
int16 mux = 0;
int16 mux_pv = 0;
int16 harm = 0;
int16 n_amostras = 0;
int16 counter_proc = counter_process;
int16 time_getdata = 0;

int TIME_DETECTION = 0;

int send_position = 0;
int16 send_n_channels = ChannelsRead;
int16 send_ch_pv;

int16 contadordeteste  = 10;
int16 teste_counter;

_Bool FAI_detect = 0;
int FAI_fase_detect[3]; 

Uint16  time_FAI = 0;


float freq_amostragem = 0;
int last_ch_pvR;
// FLAGS e Contadores
Uint16 counter_flag = 0;
// Variaveis utilizadas para transmição e configuração de HARDWARE
Uint16 transmit_a = 0;
Uint16 transmit_b = 0;
// Variaveis de Relogio
Uint16 global_timer = 0;
Uint16 global_seconds = 0;
Uint16 global_minutes = 0;
Uint16 global_hours = 0;
// Variaveis dos TIMERS
Uint16 timerzero_t_us = 10000;
Uint16 timerum_t_us = 10;
Uint16 timerdois_t_us = 1000;
Uint16 timerum_t_us_new = 10;    // Valor de atualização do TIMER1 de amostragem

//Variaveis AUXILIARES e de teste
Uint16 rdata;
Uint16 rdata1;
Uint16 rdata2;

_Bool process_flag = 0;
_Bool Conversion_Done_a = 0;
_Bool Conversion_Done_b = 0;
_Bool Update_timerum = 0;
_Bool enable_update = 1;
_Bool flag_zera_tempo = 0;
_Bool amostragem_flag = Amostragen_flag;
_Bool obtendo_dados = 0;
_Bool enable_send_FAI = 1;




struct dataandlen{
    int len;
    int16 *vector_send;   // Ponteiro utilizado para enviar DADOS de 16bits
};

struct dataandlen datalen;



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//////// SOFTWARE  SOFTWARE  SOFTWARE  SOFTWARE  SOFTWARE   SOFTWARE      //////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void iniciaprocesso(void){
    // funções que calculam o tempo de CPU utilizado no processo principal
    //    GpioDataRegs.GPCSET.bit.GPIO92 = 1;
          timercontrol = 0;
          iniciotimer = (float)((0.01*timerum_t_us)-0.005*CpuTimer1Regs.TIM.all);
          freq_amostragem = 100000000.0/(timerum_t_us);
}
void terminaprocesso(void){
    // funções que calculam o tempo de CPU utilizado no processo principal
            fimtimer = (float)((0.01*timerum_t_us)-0.005*CpuTimer1Regs.TIM.all);
            timersave = (((float)timercontrol)+fimtimer-iniciotimer);
            CPU_percent = ((float)(0.01*counter_proc*timerum_t_us))-timersave;
#if metodo==4
            CPU_percent = counter_proc*100*(1-CPU_percent/(0.01*counter_proc*timerum_t_us));
#else
            CPU_percent = 100*(1-CPU_percent/(0.01*counter_proc*timerum_t_us));
#endif
}

/*******************************************************************************
 * ********************* funcoes de morfologia     *****************************
 * ****************************************************************************/
#if metodo==3
float dilatacao_i(int pvMM, int length,const float *SE_vector, int fase, float base, int atraso_w){

    float auxiliar;
    float max;
    int jMM;
    int iMM;

    jMM = pvMM - atraso_w;
    if (jMM<0) jMM += FFTLenWindow*amostragem*Nciclos;

    max = DATA_V_I_BASE[fase]*DATA_VECTOR[fase][jMM]*base + SE_vector[0];

    for (iMM=1; iMM<length; iMM++){
        jMM = pvMM - iMM - atraso_w;
        if (jMM<0) jMM += FFTLenWindow*amostragem*Nciclos;
        auxiliar = DATA_V_I_BASE[fase]*DATA_VECTOR[fase][jMM]*base + SE_vector[iMM];
        if (auxiliar > max) max = auxiliar;

    }
    return max;
}
float erosao_i(int pvMM, int length,const float *SE_vector, int fase, float base, int atraso_w){

    float auxiliar;
    float min;
    int jMM;
    int iMM;

    jMM = pvMM - atraso_w;
    if (jMM<0) jMM += FFTLenWindow*amostragem*Nciclos;

    min = DATA_V_I_BASE[fase]*DATA_VECTOR[fase][jMM]*base - SE_vector[0];

    for (iMM=1; iMM<length; iMM++){
        jMM = pvMM + iMM - atraso_w;
        if (jMM>=FFTLenWindow*amostragem*Nciclos) jMM -= FFTLenWindow*amostragem*Nciclos;
        auxiliar = DATA_V_I_BASE[fase]*DATA_VECTOR[fase][jMM]*base - SE_vector[iMM];
        if (auxiliar < min) min = auxiliar;

    }
    return min;
}
float dilatacao_f( int V_len, float vector[][V_len], int pvMM, int length, float *SE_vector, int fase, int atraso_w){

    float auxiliar;
    float max;
    int jMM;
    int iMM;


	jMM = pvMM - atraso_w;
    if (jMM<0) jMM += V_len;

    max = vector[fase][jMM] + SE_vector[0];

    for (iMM=1; iMM<length; iMM++){
        jMM = pvMM - iMM - atraso_w;
        if (jMM<0) jMM += V_len;
        auxiliar = vector[fase][jMM] + SE_vector[iMM];
        if (auxiliar > max) max = auxiliar;

    }
    return max;
}
float erosao_f(int V_len, float vector[][V_len], int pvMM, int length, float *SE_vector, int fase, int atraso_w){

    float auxiliar;
    float min;
    int jMM;
    int iMM;

	
	jMM = pvMM - atraso_w;
    if (jMM<0) jMM += V_len;

    min = vector[fase][jMM] - SE_vector[0];

    for (iMM=1; iMM<length; iMM++){
        jMM = pvMM + iMM - atraso_w;
        if (jMM<0) jMM += V_len;
        if (jMM>=V_len) jMM -= V_len;
        auxiliar = vector[fase][jMM] - SE_vector[iMM];
        if (auxiliar < min) min = auxiliar;

    }
    return min;
}
#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////
#if metodo==4
float w_dilatacao_i(int pvMM, int length, float *SE_vector, int fase, float base, int atraso_w){

    float auxiliar;
    float max;
    int jMM;
    int iMM;

    jMM = pvMM - atraso_w;
    if (jMM<0) jMM += FFTLenWindow*amostragem*Nciclos;

    max = DATA_V_I_BASE[fase]*DATA_VECTOR[fase][jMM]*base * SE_vector[0];

    for (iMM=1; iMM<length; iMM++){
        jMM = pvMM - iMM - atraso_w;
        if (jMM<0) jMM += FFTLenWindow*amostragem*Nciclos;
        auxiliar = DATA_V_I_BASE[fase]*DATA_VECTOR[fase][jMM]*base * SE_vector[iMM];
        if (auxiliar > max) max = auxiliar;

    }
    return max;
}
float w_erosao_i(int pvMM, int length, float *SE_vector, int fase, float base, int atraso_w){

    float auxiliar;
    float min;
    int jMM;
    int iMM;

    jMM = pvMM - atraso_w;
    if (jMM<0) jMM += FFTLenWindow*amostragem*Nciclos;

    min = DATA_V_I_BASE[fase]*DATA_VECTOR[fase][jMM]*base / SE_vector[0];

    for (iMM=1; iMM<length; iMM++){
        jMM = pvMM + iMM - atraso_w;
        if (jMM>=FFTLenWindow*amostragem*Nciclos) jMM -= FFTLenWindow*amostragem*Nciclos;
        auxiliar = DATA_V_I_BASE[fase]*DATA_VECTOR[fase][jMM]*base / SE_vector[iMM];
        if (auxiliar < min) min = auxiliar;

    }
    return min;
}
float w_dilatacao_f(int V_len, float vector[][V_len], int pvMM, int length, float *SE_vector, int fase, int atraso_w){

    float auxiliar;
    float max;
    int jMM;
    int iMM;


	jMM = pvMM - atraso_w;
    if (jMM<0) jMM += V_len;

    max = vector[fase][jMM] * SE_vector[0];

    for (iMM=1; iMM<length; iMM++){
        jMM = pvMM - iMM - atraso_w;
        if (jMM<0) jMM += V_len;
        auxiliar = vector[fase][jMM] * SE_vector[iMM];
        if (auxiliar > max) max = auxiliar;

    }
    return max;
}
float w_erosao_f(int V_len, float vector[][V_len], int pvMM, int length, float *SE_vector, int fase, int atraso_w){

    float auxiliar;
    float min;
    int jMM;
    int iMM;

	
	jMM = pvMM - atraso_w;
    if (jMM<0) jMM += V_len;

    min = vector[fase][jMM] / SE_vector[0];

    for (iMM=1; iMM<length; iMM++){
        jMM = pvMM + iMM - atraso_w;
        if (jMM<0) jMM += V_len;
        if (jMM>=V_len) jMM -= V_len;
        auxiliar = vector[fase][jMM] / SE_vector[iMM];
        if (auxiliar < min) min = auxiliar;

    }
    return min;
}
#endif
// fim dos algoritmos de MM

/*******************************************************************************
 * ********************* algoritmo do brhama       *****************************
 * ****************************************************************************/
#if metodo==3
void brhama2012(void){

iniciaprocesso();


    Br_dilatacao[Va][MMpv] = dilatacao_i(pv, MMSELEN, Br_SE, VA, 0.0000833333, 1);
    Br_erosao[Va][MMpv] = erosao_i(pv, MMSELEN, Br_SE, VA, 0.0000833333, 1);

    Br_dilatacao[Vb][MMpv] = dilatacao_i(pv, MMSELEN, Br_SE, VB, 0.0000833333, 1);
    Br_erosao[Vb][MMpv] = erosao_i(pv, MMSELEN, Br_SE, VB, 0.0000833333, 1);

    Br_dilatacao[Vc][MMpv] = dilatacao_i(pv, MMSELEN, Br_SE, VC, 0.0000833333, 1);
    Br_erosao[Vc][MMpv] = erosao_i(pv, MMSELEN, Br_SE, VC, 0.0000833333, 1);

    Br_opened[Va][MMpv] = dilatacao_f(Br_len, Br_erosao, MMpv, MMSELEN, Br_SE, Va, 1);
    Br_closed[Va][MMpv] = erosao_f(Br_len, Br_dilatacao, MMpv, MMSELEN, Br_SE, Va, 1);

    Br_opened[Vb][MMpv] = dilatacao_f(Br_len, Br_erosao, MMpv, MMSELEN, Br_SE, Vb, 1);
    Br_closed[Vb][MMpv] = erosao_f(Br_len, Br_dilatacao, MMpv, MMSELEN, Br_SE, Vb, 1);

    Br_opened[Vc][MMpv] = dilatacao_f(Br_len, Br_erosao, MMpv, MMSELEN, Br_SE, Vc, 1);
    Br_closed[Vc][MMpv] = erosao_f(Br_len, Br_dilatacao, MMpv, MMSELEN, Br_SE, Vc, 1);

    CODO[Va][CODOpv] = Br_closed[Va][MMpv] - Br_opened[Va][MMpv];
    CODO[Vb][CODOpv] = Br_closed[Vb][MMpv] - Br_opened[Vb][MMpv];
    CODO[Vc][CODOpv] = Br_closed[Vc][MMpv] - Br_opened[Vc][MMpv];

    if(prave<CODO[Va][CODOpv]) prave = CODO[Va][CODOpv];

    DATA_VECTOR[ChannelsRead-1][pv] = (int16)(CODO[Va][CODOpv]*1000);

    if(CODO[Va][CODOpv] >= Br_th[Va]){
    	if(Br_inicialset[Va] && Br_time[Va]>Br_tw && Br_time[Va]<1000){
    		Br_FAI_detect[Va] = true;
    		FAI_detect = true;
        FAI_fase_detect[0] = 1;
		}
		if(Br_inicialset[Va] == 0) Br_time[Va] = 0;
		Br_inicialset[Va] = true;
    }
    if(Br_time[Va] > 1000){
    	Br_inicialset[Va] = false;
    	Br_time[Va] = 0;
    	Br_FAI_detect[Va] = false;
    }

    if(CODO[Vb][CODOpv] >= Br_th[Vb]){
    	if(Br_inicialset[Vb] && Br_time[Vb]>Br_tw && Br_time[Vb]<1000){
    		Br_FAI_detect[Vb] = true;
    		FAI_detect = true;
        FAI_fase_detect[1] = 1;
		}
		if(Br_inicialset[Vb] == 0) Br_time[Vb] = 0;
		Br_inicialset[Vb] = true;
    }
    if(Br_time[Vb] > 1000){
    	Br_inicialset[Vb] = false;
    	Br_time[Vb] = 0;
    	Br_FAI_detect[Vb] = false;
    }

    if(CODO[Vc][CODOpv] >= Br_th[Vc]){
    	if(Br_inicialset[Vc] && Br_time[Vc]>Br_tw && Br_time[Vc]<1000){
    		Br_FAI_detect[Vc] = true;
    		FAI_detect = true;
        FAI_fase_detect[2] = 1;
		}
		if(Br_inicialset[Vc] == 0) Br_time[Vc] = 0;
		Br_inicialset[Vc] = true;
    }
    if(Br_time[Vc] > 1000){
    	Br_inicialset[Vc] = false;
    	Br_time[Vc] = 0;
    	Br_FAI_detect[Vc] = false;
    }

    if(MMpv>=Br_len-1) MMpv = 0;
    else MMpv++;

    if(CODOpv>=Br_len_C-1) CODOpv = 0;
    else CODOpv++;

terminaprocesso();
}
#endif
// fim do algoritmo do brhama
/*******************************************************************************
 * ********************* algoritmo do kavi2018      ****************************
 * ****************************************************************************/
#if metodo==4
void kavi_mmf(int fase){

	int mmf_pv = Ka_mmf_pv - 2;
	if(mmf_pv<0) mmf_pv += Ka_mmf_len;

	Ka_mmf_DI1[fase][Ka_mmf_pv] = w_dilatacao_i(pv, 3, Ka_A1, fase, 1.0/DATA_V_I_RTC_RTP[fase], 2);
	Ka_mmf_EI1[fase][Ka_mmf_pv] = w_erosao_i(pv, 3, Ka_A1, fase, 1.0/DATA_V_I_RTC_RTP[fase], 2);

	Ka_mmf_DI2[fase][Ka_mmf_pv] = w_dilatacao_f( Ka_mmf_len, Ka_mmf_DI1, Ka_mmf_pv, 3, Ka_A2, fase, 2);
	Ka_mmf_EI2[fase][Ka_mmf_pv] = w_erosao_f( Ka_mmf_len, Ka_mmf_EI1, Ka_mmf_pv, 3, Ka_A2, fase, 2);

	Ka_mmf_Av[fase] = (Ka_mmf_DI1[fase][mmf_pv] + Ka_mmf_DI2[fase][Ka_mmf_pv] + Ka_mmf_EI1[fase][mmf_pv] + Ka_mmf_EI2[fase][Ka_mmf_pv])/4;

}

void kavi_diff(int fase, float base){

	int pv_aux;

	pv_aux = pv - 4;
    if (pv_aux<0) pv_aux += FFTLenWindow*amostragem*Nciclos;

	Ka_dif[fase][Ka_dif_pv] = Ka_mmf_Av[fase] - DATA_V_I_BASE[fase]*DATA_VECTOR[fase][pv_aux]*base;

}

void kavi_asf(int fase){

// OC
	Ka_asf_OC_1[fase][Ka_asf_pv] = w_dilatacao_f(Ka_dif_len, Ka_dif, Ka_dif_pv, 5, Ka_B2, fase, 4);
	Ka_asf_OC_2[fase][Ka_asf_pv] = w_erosao_f(Ka_asf_len, Ka_asf_OC_1, Ka_asf_pv, 5, Ka_B2, fase, 4);

	Ka_asf_OC_3[fase][Ka_asf_pv] = w_erosao_f(Ka_asf_len, Ka_asf_OC_2, Ka_asf_pv, 5, Ka_B2, fase, 4);
	Ka_asf_OC_4[fase][Ka_asf_pv] = w_dilatacao_f(Ka_asf_len, Ka_asf_OC_3, Ka_asf_pv, 5, Ka_B2, fase, 4);

	Ka_asf_OC_5[fase][Ka_asf_pv] = w_dilatacao_f(Ka_asf_len, Ka_asf_OC_4, Ka_asf_pv, 3, Ka_B1, fase, 2);
	Ka_asf_OC_6[fase][Ka_asf_pv] = w_erosao_f(Ka_asf_len, Ka_asf_OC_5, Ka_asf_pv, 3, Ka_B1, fase, 2);

	Ka_asf_OC_7[fase][Ka_asf_pv] = w_erosao_f(Ka_asf_len, Ka_asf_OC_6, Ka_asf_pv, 3, Ka_B1, fase, 2);
	Ka_asf_OC_8[fase][Ka_asf_pv] = w_dilatacao_f(Ka_asf_len, Ka_asf_OC_7, Ka_asf_pv, 3, Ka_B1, fase, 2);


// CO
	Ka_asf_CO_1[fase][Ka_asf_pv] = w_erosao_f(Ka_dif_len, Ka_dif, Ka_dif_pv, 5, Ka_B2, fase, 4);
	Ka_asf_CO_2[fase][Ka_asf_pv] = w_dilatacao_f(Ka_asf_len, Ka_asf_CO_1, Ka_asf_pv, 5, Ka_B2, fase, 4);

	Ka_asf_CO_3[fase][Ka_asf_pv] = w_dilatacao_f(Ka_asf_len, Ka_asf_CO_2, Ka_asf_pv, 5, Ka_B2, fase, 4);
	Ka_asf_CO_4[fase][Ka_asf_pv] = w_erosao_f(Ka_asf_len, Ka_asf_CO_3, Ka_asf_pv, 5, Ka_B2, fase, 4);

	Ka_asf_CO_5[fase][Ka_asf_pv] = w_erosao_f(Ka_asf_len, Ka_asf_CO_4, Ka_asf_pv, 3, Ka_B1, fase, 2);
	Ka_asf_CO_6[fase][Ka_asf_pv] = w_dilatacao_f(Ka_asf_len, Ka_asf_CO_5, Ka_asf_pv, 3, Ka_B1, fase, 2);

	Ka_asf_CO_7[fase][Ka_asf_pv] = w_dilatacao_f(Ka_asf_len, Ka_asf_CO_6, Ka_asf_pv, 3, Ka_B1, fase, 2);
	Ka_asf_CO_8[fase][Ka_asf_pv] = w_erosao_f(Ka_asf_len, Ka_asf_CO_7, Ka_asf_pv, 3, Ka_B1, fase, 2);

}

void kavi_mdf(int fase){

	int asf_pv_aux3 = Ka_asf_pv - 4;
	int asf_pv_aux2 = Ka_asf_pv - 8;
	int asf_pv_aux1 = Ka_asf_pv - 16;

	if(asf_pv_aux3<0) asf_pv_aux3 += Ka_asf_len;
	if(asf_pv_aux2<0) asf_pv_aux2 += Ka_asf_len;
	if(asf_pv_aux1<0) asf_pv_aux1 += Ka_asf_len;

	Ka_mdf[fase][Ka_mdf_pv] = ( (Ka_asf_OC_2[fase][asf_pv_aux1] - Ka_asf_CO_2[fase][asf_pv_aux1]) + (Ka_asf_OC_4[fase][asf_pv_aux2] - Ka_asf_CO_4[fase][asf_pv_aux2]) + (Ka_asf_CO_6[fase][asf_pv_aux3] - Ka_asf_OC_6[fase][asf_pv_aux3]) + (Ka_asf_OC_8[fase][Ka_asf_pv] - Ka_asf_CO_8[fase][Ka_asf_pv]) );

}

void kavi2018(void){

iniciaprocesso();


	if(Ka_mmf_pv >= Ka_mmf_len-1) Ka_mmf_pv = 0;
	else Ka_mmf_pv++;

    kavi_mmf(Ka_fase);
//    kavi_mmf(IB);
//    kavi_mmf(IC);
//	  kavi_mmf(VA);
//    kavi_mmf(VB);
//    kavi_mmf(VC);



	if(Ka_dif_pv >= Ka_dif_len-1) Ka_dif_pv = 0;
	else Ka_dif_pv++;


    kavi_diff(Ka_fase, 1.0/DATA_V_I_RTC_RTP[Ka_fase]);
//    kavi_diff(IB, 1.0/DATA_V_I_BASE[IB]);
//    kavi_diff(IC, 1.0/DATA_V_I_BASE[IC]);
//	kavi_diff(VA, 1.0/DATA_V_I_BASE[VA]);
//    kavi_diff(VB, 1.0/DATA_V_I_BASE[VB]);
//    kavi_diff(VC, 1.0/DATA_V_I_BASE[VC]);

	if(Ka_asf_pv >= Ka_asf_len-1) Ka_asf_pv = 0;
	else Ka_asf_pv++;

	kavi_asf(Ka_fase);
//	kavi_asf(IB);
//	kavi_asf(IC);
//	kavi_asf(VA);
//	kavi_asf(VB);
//	kavi_asf(VC);

	if(Ka_mdf_pv >= Ka_mdf_len-1) Ka_mdf_pv = 0;
	else Ka_mdf_pv++;

	kavi_mdf(Ka_fase);
//	kavi_mdf(IB);
//	kavi_mdf(IC);
//	kavi_mdf(VA);
//	kavi_mdf(VB);
//	kavi_mdf(VC);


///////////////////
	if (frequencia < 65 && frequencia > 55 && fftS[0][1] > 10 && !Ka_flag_first_detection){
	    if (Ka_counter < 68){
	            Ka_counter++;
	        }else{
	            Ka_counter = 0;
	        }
	    if(Ka_inicio){
            if((Ka_find_tall < Ka_mdf[Ka_fase][Ka_mdf_pv])){
                Ka_find_tall = Ka_mdf[Ka_fase][Ka_mdf_pv];
                Ka_find_tall_index = Ka_counter;
            }
            if (Ka_counter==63){
                Ka_inicio = false;
                Ka_tall = Ka_find_tall;
                Ka_index = Ka_find_tall_index;
                Ka_counter = Ka_counter - Ka_find_tall_index;
            }
	    }else{
            if((Ka_find_tall < Ka_mdf[Ka_fase][Ka_mdf_pv])&&(Ka_counter>58)&&(Ka_counter<68)){
                Ka_find_tall = Ka_mdf[Ka_fase][Ka_mdf_pv];
                Ka_find_tall_index = Ka_counter;
            }
            if(Ka_counter==68){
                if((Ka_find_tall_index-Ka_index)==0){
                    Ka_flag_tall_rand = false;
                }else{
                    Ka_flag_tall_rand = true;
                }
                if(Ka_fase == IA && Ka_find_tall != 0){
                    Ka_Iinc = (Ka_find_tall - Ka_tall)/Ka_tall;
                    if(Ka_Iinc > 1){
                        Ka_flag_first_detection = true;
                        Ka_fase = VA;
                        Ka_counter_16ms = 0;
                    }else{
                        Ka_flag_first_detection = false;
                    }
                    Ka_tall = Ka_find_tall;
                    Ka_index = Ka_find_tall_index;
                    Ka_counter = Ka_counter - Ka_find_tall_index;
                }

                Ka_find_tall = 0;
            }

	    }
	}else{
	    Ka_inicio = true;
	    Ka_counter = 0;
	}
	if (Ka_counter_16ms >= 16){
	    Ka_counter_16ms = 0;
	    Ka_counter_1s = 0;
	    Ka_flag_first_detection = 0;
	    Ka_second_detection = true;
	}
	if(Ka_counter_1s >= 1000){
	    Ka_second_detection = 0;
	    Ka_counter_1s = 0;
	    Ka_fase = IA;
        Ka_inicio = true;
        Ka_counter = 0;
	}
	/*
if(Ka_flag_counter[IA]==0){

	Ka_Iinc[IA] = (Ka_Imax[IA] - Ka_mdf[IA][Ka_mdf_pv]) / Ka_mdf[IA][Ka_mdf_pv];

	if(Ka_Iinc[IA] < 0.575){
		Ka_Imax[IA] = Ka_mdf[IA][Ka_mdf_pv];
	}else if(Ka_Iinc[IA] > 1){
		Ka_flag_L1 = true;
	}else{
		Ka_flag_counter[IA] = true;;
	}
}else{
	Ka_counter_ini[IA]++;
	if(Ka_counter_ini[IA] > 17){
		Ka_flag_1s[IA] = true;
		if(Ka_counter_1s[IA] > 1000){
			Ka_counter_ini[IA] = 0;
			Ka_flag_counter[IA] = false;
			Ka_counter_1s[IA] = 0;
			Ka_flag_1s[IA] = false;





			
		}
	}
}
*/
//////////////////
terminaprocesso();
}
#endif
// fim do algoritmo do kavi

/*******************************************************************************
 *              CONSTROI A FASE
 ******************************************************************************/
float phase_biuld(Uint16 ch_pv, Uint16 harm, int faseref){
    float gallas;
    float angle_biuld;

    gallas = (fftA[ch_pv][harm]-90);
    if(gallas<=-180.0){
        gallas += 360;
    }
    angle_biuld = (fftA[faseref][1]-90);
    if(angle_biuld<=-180.0){
        angle_biuld += 360;
    }
    angle_biuld = angle_biuld * harm;
    while(angle_biuld>=180.0){
        angle_biuld -= 360;
    }
    while(angle_biuld<=-180.0){
        angle_biuld += 360;
    }
    gallas = gallas - angle_biuld;
    if(gallas>=180.0){
            gallas=gallas-360;
    }
    if(gallas<=-180.0){
            gallas=gallas+360;
    }
    if((gallas>=-0.0001)&&(gallas<=0.0001)){
        gallas = 0;
    }
    return gallas;
}

/*******************************************************************************
 * *****************************************************************************
 * ****************************************************************************/
/*******************************************************************************
 *          CALCULA O FFT
 ******************************************************************************/
void fft_complete(Uint16 ch_pv, Uint16 harm){

    float fftR = 0;
    float fftI = 0;
    int i;

  for(i=0;i<FFTLenWindow*amostragem;i++){

    fftR += DATA_VECTOR_HOLD[ch_pv][i]*Cfc[harm][i];
    fftI += DATA_VECTOR_HOLD[ch_pv][i]*Cfs[harm][i];
  }
    fftR = constant_dois*fftR;
    fftI = constant_dois*fftI;

    fftS[ch_pv][harm] = sqrt(fftR*fftR+(fftI*fftI));
    fftA[ch_pv][harm] = atan2(fftI,fftR)*57.295779;

}
/*******************************************************************************
 * ****************metodos de deteccao baseados em harmonicos ******************
 * ****************************************************************************/
void executa_FFT(void){
iniciaprocesso();
////****************************************************************************
//******************************************************************************
  counter_flag = 0;
  pv_hold = pv;

  int k;
  int jj;
  int i;

  for(k=0;k<FFTLenWindow*amostragem;k++){

  if((pv_hold+k-FFTLenWindow*amostragem)<0){
      jj = pv_hold+k-FFTLenWindow*amostragem+FFTLenWindow*amostragem*Nciclos;
  }else{
      jj = pv_hold+k-FFTLenWindow*amostragem;
  }

  for(i=0;i<ChannelsRead;i++){
      DATA_VECTOR_HOLD[i][k] = DATA_V_I_BASE[i]*DATA_VECTOR[i][jj];
  }
  }
/** Calcula a FFT para todos os harmonicos contidos em HarmNum                */
    for(k=0;k<HarmNum;k++){
        for(j=0;j<ChannelsRead;j++){
            fft_complete(j,k);
        }
    }
/** Constroi a fase para todos os harmonicos                                                        */
    for(k=0;k<HarmNum;k++){
        for(j=0;j<ChannelsRead;j++){
           phase[j][k] = phase_biuld(j,k,0);
        }
    }

terminaprocesso();
//caso estore o uso da CPU será incrementado o counter_process para resolver o problema
if(CPU_percent>=99){
    counter_proc++;
}
}



float * pol2cart(float th, float r){
    static float conv_p[2];

    th = th*0.0174532925;

    conv_p[0] = r*cos(th);
    conv_p[1] = r*sin(th);

    return conv_p;
}
float * cart2pol(float x, float y){
    static float conv_c[2];

    conv_c[0] = atan2(y,x)*57.295779;
    conv_c[1] = hypot(x,y);

    return conv_c;
}

float absvalue(float x){
    if(x<0)
        x = x*-1;

    return x;
}

float * mediadafasor(float modulo, float angulo, float modulo_M, float angulo_M, int multiplicador){
   static float conv_pa[2];
    float *temp_m;
    float x;
    float y;

    temp_m = pol2cart(angulo,modulo);
    x = temp_m[0];
    y = temp_m[1];
    temp_m = pol2cart(angulo_M,modulo_M*multiplicador);
    x = x+temp_m[0];
    y = y+temp_m[1];

    temp_m = cart2pol(x,y);

    conv_pa[0] = temp_m[0];
    conv_pa[1] = temp_m[1]/(multiplicador+1);

    return conv_pa;
}
float * subtraifasor(float modulo1, float angulo1, float modulo2, float angulo2){
   static float conv_su[2];
    float *temp_m;
    float x;
    float y;

    temp_m = pol2cart(angulo1,modulo1);
    x = temp_m[0];
    y = temp_m[1];
    temp_m = pol2cart(angulo2,modulo2);
    x = x-temp_m[0];
    y = y-temp_m[1];

    temp_m = cart2pol(x,y);

    conv_su[0] = temp_m[0];
    conv_su[1] = temp_m[1];

    return conv_su;
}

#if metodo==2

float mediadafase(int fase){
    float *temp_m;
    float x;
    float y;
    float conv_m;

    temp_m = pol2cart(phase[fase][3],1);
    x = temp_m[0];
    y = temp_m[1];
    temp_m = pol2cart(Alda_faseM[fase-3],Alda_divisor-1);
    x = x+temp_m[0];
    y = y+temp_m[1];

    temp_m = cart2pol(x,y);

    conv_m = temp_m[0];

    return conv_m;
}
void calculacompenentesdesequencia(void){
    float *temp;
    double complex Ia_c;
    double complex Ib_c;
    double complex Ic_c;
    double complex a1fq;
    double complex a2fq;
    double complex Isqn;
    double complex Isqp;


    a2fq = -0.5 + 0.866025403784439*I;
    a1fq = -0.5 - 0.866025403784439*I;

    temp = pol2cart(phase[IA][1],fftS[IA][1]);
    Ia_c = temp[0] + temp[1]*I;
    temp = pol2cart(phase[IB][1],fftS[IB][1]);
    Ib_c = temp[0] + temp[1]*I;
    temp = pol2cart(phase[IC][1],fftS[IC][1]);
    Ic_c = temp[0] + temp[1]*I;

    Isqn = (Ia_c + a1fq*Ib_c+ a2fq*Ic_c) / 3;
    Isqp = (Ia_c + a2fq*Ib_c+ a1fq*Ic_c) / 3;

    temp = cart2pol(creal(Isqp),cimag(Isqp));
    Alda_I1m = temp[1];
    temp = cart2pol(creal(Isqn),cimag(Isqn));
    Alda_I2m = temp[1];

}
/*******************************************************************************
 *          DETECTOR DO ALDAIR
 ******************************************************************************/
void Aldair2020(void){
iniciaprocesso();
////****************************************************************************
    counter_flag = 0;
    pv_hold = pv;

    float *Alda_temp;

    int k;
    int jj;
    int i;

    for(k=0;k<FFTLenWindow*amostragem;k++){
      if((pv_hold+k-FFTLenWindow*amostragem)<0){
          jj = pv_hold+k-FFTLenWindow*amostragem+FFTLenWindow*amostragem*Nciclos;
      }else{
          jj = pv_hold+k-FFTLenWindow*amostragem;
      }
      for(i=0;i<ChannelsRead;i++){
          DATA_VECTOR_HOLD[i][k] = DATA_V_I_BASE[i]*DATA_VECTOR[i][jj];
      }
    }

    fft_complete(VA,1);

    fft_complete(IN,1);
    fft_complete(IN,2);
    fft_complete(IN,3);
    fft_complete(IN,4);
    fft_complete(IN,5);
    fft_complete(IN,6);
    fft_complete(IN,7);

    fft_complete(IA,1);
    fft_complete(IB,1);
    fft_complete(IC,1);

    fft_complete(IA,3);
    fft_complete(IB,3);
    fft_complete(IC,3);

    phase[IA][1] = phase_biuld(IA,1,VA);
    phase[IB][1] = phase_biuld(IB,1,VA);
    phase[IC][1] = phase_biuld(IC,1,VA);

    phase[IA][3] = phase_biuld(IA,3,VA);
    phase[IB][3] = phase_biuld(IB,3,VA);
    phase[IC][3] = phase_biuld(IC,3,VA);

    phase[IN][1] = phase_biuld(IN,1,VA);
    phase[IN][3] = phase_biuld(IN,3,VA);

    phaseIA[Ib] = phase_biuld(IB,1,IA);
    phaseIA[Ic] = phase_biuld(IC,1,IA);

    phaseIB[Ia] = phase_biuld(IA,1,IB);
    phaseIB[Ic] = phase_biuld(IC,1,IB);

    phaseIC[Ia] = phase_biuld(IA,1,IC);
    phaseIC[Ib] = phase_biuld(IB,1,IC);
/**
* My detector
*/
////****************************************************************************
    int temp = round(amostragem/counter_proc);

    Alda_divisor = Alda_nciclos*temp;
    Alda_divisor_fasor = Alda_nciclos_fasor*temp;

    Alda_I2mant2 = Alda_I2mant1;
    Alda_I1mant2 = Alda_I1mant1;

    Alda_I2mant1 = Alda_I2mant;
    Alda_I1mant1 = Alda_I1mant;

    Alda_I2mant = Alda_I2m;
    Alda_I1mant = Alda_I1m;

    calculacompenentesdesequencia();

    Alda_I2m = (Alda_I2m+3*Alda_I2mant)/4;
    Alda_I1m = (Alda_I1m+3*Alda_I1mant)/4;

    Alda_der_I2m = 1000000.0*counter_proc*absvalue(Alda_I2m-Alda_I2mant)/(amostragem*timerum_t_us);

    if(fftS[IA][1]>10 && fftS[IB][1]>10 && fftS[IC][1]>10){

         if(Alda_der_I2m>200 && Alda_flag_CBs==0){
             Alda_I1_holdi = Alda_I1mant2;
             Alda_I2_holdi = Alda_I2mant2;
             Alda_Ia_holdi   = Alda_Ia_holdiant3;
             Alda_fb2a_holdi = Alda_fb2a_holdiant3;
             Alda_fc2a_holdi = Alda_fc2a_holdiant3;
             Alda_Ib_holdi   = Alda_Ib_holdiant3;
             Alda_fa2b_holdi = Alda_fa2b_holdiant3;
             Alda_fc2b_holdi = Alda_fc2b_holdiant3;
             Alda_Ic_holdi   = Alda_Ic_holdiant3;
             Alda_fa2c_holdi = Alda_fa2c_holdiant3;
             Alda_fb2c_holdi = Alda_fb2c_holdiant3;
             Alda_flag_CBs = 1;
         }
         if ((Alda_flag_CBs==1)&&(Alda_der_I2m<400)){
             Alda_count_CBs = Alda_count_CBs + 1;
         }else{
             Alda_count_CBs = 0;
         }
         if (Alda_count_CBs>=Alda_divisor){
             Alda_I1_holdf = Alda_I1m;
             Alda_I2_holdf = Alda_I2m;
             Alda_Ia_holdf = fftS[IA][1];
             Alda_fb2a_holdf = absvalue(phaseIA[Ib]);
             Alda_fc2a_holdf = absvalue(phaseIA[Ic]);
             Alda_Ib_holdf = fftS[IB][1];
             Alda_fa2b_holdf = absvalue(phaseIB[Ia]);
             Alda_fc2b_holdf = absvalue(phaseIB[Ic]);
             Alda_Ic_holdf = fftS[IC][1];
             Alda_fa2c_holdf = absvalue(phaseIC[Ia]);
             Alda_fb2c_holdf = absvalue(phaseIC[Ib]);
             Alda_flag_CBs = 0;

             Alda_delta_Ia = Alda_Ia_holdi-Alda_Ia_holdf;
             Alda_delta_Ib = Alda_Ib_holdi-Alda_Ib_holdf;
             Alda_delta_Ic = Alda_Ic_holdi-Alda_Ic_holdf;

             if((Alda_I1_holdi-Alda_I1_holdf)>(0.01*Alda_I1_holdi)&&(Alda_I2_holdf-Alda_I2_holdi)>(0.01*Alda_I2_holdi)){

                if((Alda_delta_Ia>Alda_delta_Ib)&&(Alda_delta_Ia>Alda_delta_Ic)){
                   Alda_CBsf[Ia] = 1;
                   Alda_CBs=1;
                   Alda_reset_CB = 0;
                }
                if((Alda_delta_Ib>Alda_delta_Ia)&&(Alda_delta_Ib>Alda_delta_Ic)){
                   Alda_CBsf[Ib] = 1;
                   Alda_CBs=1;
                   Alda_reset_CB = 0;
                }
                if((Alda_delta_Ic>Alda_delta_Ia)&&(Alda_delta_Ic>Alda_delta_Ib)){
                   Alda_CBsf[Ic] = 1;
                   Alda_CBs=1;
                   Alda_reset_CB = 0;
                }
             }

         }
         if (Alda_I2m>0.2*Alda_I1m){
             Alda_CBsp = 1;
         }else{
             Alda_CBsp = 0;
         }
    }

    Alda_Ia_holdiant3 = Alda_Ia_holdiant2;
    Alda_fb2a_holdiant3 = Alda_fb2a_holdiant2;
    Alda_fc2a_holdiant3 = Alda_fc2a_holdiant2;
    Alda_Ib_holdiant3 = Alda_Ib_holdiant2;
    Alda_fa2b_holdiant3 = Alda_fa2b_holdiant2;
    Alda_fc2b_holdiant3 = Alda_fc2b_holdiant2;
    Alda_Ic_holdiant3 = Alda_Ic_holdiant2;
    Alda_fa2c_holdiant3 = Alda_fa2c_holdiant2;
    Alda_fb2c_holdiant3 = Alda_fb2c_holdiant2;

    Alda_Ia_holdiant2 = Alda_Ia_holdiant1;
    Alda_fb2a_holdiant2 = Alda_fb2a_holdiant1;
    Alda_fc2a_holdiant2 = Alda_fc2a_holdiant1;
    Alda_Ib_holdiant2 = Alda_Ib_holdiant1;
    Alda_fa2b_holdiant2 = Alda_fa2b_holdiant1;
    Alda_fc2b_holdiant2 = Alda_fc2b_holdiant1;
    Alda_Ic_holdiant2 = Alda_Ic_holdiant1;
    Alda_fa2c_holdiant2 = Alda_fa2c_holdiant1;
    Alda_fb2c_holdiant2 = Alda_fb2c_holdiant1;

    Alda_Ia_holdiant1 = Alda_Ia_holdiant;
    Alda_fb2a_holdiant1 = Alda_fb2a_holdiant;
    Alda_fc2a_holdiant1 = Alda_fc2a_holdiant;
    Alda_Ib_holdiant1 = Alda_Ib_holdiant;
    Alda_fa2b_holdiant1 = Alda_fa2b_holdiant;
    Alda_fc2b_holdiant1 = Alda_fc2b_holdiant;
    Alda_Ic_holdiant1 = Alda_Ic_holdiant;
    Alda_fa2c_holdiant1 = Alda_fa2c_holdiant;
    Alda_fb2c_holdiant1 = Alda_fb2c_holdiant;

    Alda_Ia_holdiant   = fftS[IA][1];
    Alda_fb2a_holdiant = absvalue(phaseIA[Ib]);
    Alda_fc2a_holdiant = absvalue(phaseIA[Ic]);
    Alda_Ib_holdiant   = fftS[IB][1];
    Alda_fa2b_holdiant = absvalue(phaseIB[Ia]);
    Alda_fc2b_holdiant = absvalue(phaseIB[Ic]);
    Alda_Ic_holdiant   = fftS[IC][1];
    Alda_fa2c_holdiant = absvalue(phaseIC[Ia]);
    Alda_fb2c_holdiant = absvalue(phaseIC[Ib]);

////****************************************************************************
    Alda_HarmPar = fftS[6][2]+fftS[6][4]+fftS[6][6];
    Alda_HarmImpar = fftS[6][3]+fftS[6][5]+fftS[6][7];

    Alda_temp = subtraifasor(fftS[6][1],phase[6][1],Alda_fftS_media[6][1],Alda_fase_media[6][1]);

    Alda_modulo1harm = Alda_temp[1];
    Alda_angulo1harm = Alda_temp[0];

    Alda_temp = subtraifasor(fftS[6][3], 0,Alda_fftS_media[6][3],0);

    Alda_modulo3harm = Alda_temp[1];
    Alda_angulo3harm = Alda_temp[0];


    if(Alda_inicial_set==false){
        Alda_temp = mediadafasor(fftS[6][1],phase[6][1],Alda_fftS_media[6][1],Alda_fase_media[6][1],Alda_divisor_fasor);

        Alda_fftS_media[6][1] = Alda_temp[1];
        Alda_fase_media[6][1] = Alda_temp[0];

        Alda_temp = mediadafasor(fftS[6][3],0,Alda_fftS_media[6][3],0,Alda_divisor_fasor);

        Alda_fftS_media[6][3] = Alda_temp[1];
        Alda_fase_media[6][3] = Alda_temp[0];

    }
    Alda_relation = Alda_modulo3harm/Alda_modulo1harm;

    if(Alda_modulo1harm > 1 && Alda_relation > 0.02){
        if(Alda_inicial_counter > Alda_inicial_freze && Alda_firt_detection==false){
            Alda_inicial_set = false;
            Alda_inicial_counter = 0;
            Alda_fftS_media[6][1] = fftS[6][1];
            Alda_fase_media[6][1] = phase[6][1];
            Alda_fftS_media[6][3] = fftS[6][3];
            Alda_fase_media[6][3] = 0;
        }else{
            Alda_inicial_set = true;
        }
    }else{
        Alda_inicial_set = false;
        Alda_inicial_counter = 0;
    }

    if(( Alda_modulo1harm > 1) && (Alda_relation >= 0.02) && Alda_HarmImpar>Alda_HarmPar && (phase[6][3] >= 120 || phase[6][3] <= -170 || Alda_CBs))
    {
        Alda_firt_detection = true;
        Alda_detection = true;
    }else{
        Alda_detection = false;
    }

    if(Alda_firt_detection){
        if(Alda_FAI_detect_counter >= 200){
            Alda_FAI_detect = true;
            FAI_detect = true;
            Alda_FAI_detect_counter = 200;
        }
        if(Alda_FAI_detect_counter <= -200){
            Alda_FAI_detect = false;
            Alda_firt_detection = false;
            Alda_FAI_detect_counter = 0;
        }
    }
////****************************************************************************

    Alda_sfftM[Ia] = ((Alda_divisor-1)*Alda_sfftM[Ia] + fftS[IA][3])/Alda_divisor;
    Alda_sfftM[Ib] = ((Alda_divisor-1)*Alda_sfftM[Ib] + fftS[IB][3])/Alda_divisor;
    Alda_sfftM[Ic] = ((Alda_divisor-1)*Alda_sfftM[Ic] + fftS[IC][3])/Alda_divisor;

    Alda_faseM[Ia] = mediadafase(IA);
    Alda_faseM[Ib] = mediadafase(IB);
    Alda_faseM[Ic] = mediadafase(IC);

////****************************************************************************

    if(Alda_FAI_detect){
        float aux;
        float aux1;
        float aux2;

        float xua;
        float xua1;
        float xua2;

        if(Alda_sfftM[Ia]>=Alda_sfftM[Ib] && Alda_sfftM[Ia]>=Alda_sfftM[Ic]){
            aux = absvalue(Alda_faseM[Ib] - Alda_faseM[Ic]);
            if (aux > 180)  aux = 180 - (aux - 180);
            aux1 = absvalue(Alda_faseM[Ia] - Alda_faseM[Ib]);
            if (aux1 > 180) aux1 = 180 - (aux1 - 180);
            aux2 = absvalue(Alda_faseM[Ia] - Alda_faseM[Ic]);
            if (aux2 > 180) aux2 = 180 - (aux2 - 180);

            xua = absvalue(Alda_sfftM[Ib]-Alda_sfftM[Ic]);
            xua1 = absvalue(Alda_sfftM[Ia]-Alda_sfftM[Ib]);
            xua2 = absvalue(Alda_sfftM[Ia]-Alda_sfftM[Ic]);

            if(aux < aux1 && aux < aux2){
                if(aux1 > 90 && Alda_CBs){
                    Alda_classificador[Ia] = 4;
                    Alda_classificador[Ib] = 0;
                    Alda_classificador[Ic] = 0;
                }else if(Alda_CBs){
                    Alda_classificador[Ia] = 2;
                    Alda_classificador[Ib] = 0;
                    Alda_classificador[Ic] = 0;
                }else{
                    Alda_classificador[Ia] = 1;
                    Alda_classificador[Ib] = 0;
                    Alda_classificador[Ic] = 0;
                }
            }else if(aux1 < aux2 && aux1 < aux && (xua1 < xua2 && xua1 < xua)){
                Alda_classificador[Ia] = 0;
                Alda_classificador[Ib] = 0;
                Alda_classificador[Ic] = 3;
            }else if(aux2 < aux1 && aux2 < aux && (xua2 < xua1 && xua2 < xua)){
                Alda_classificador[Ia] = 0;
                Alda_classificador[Ib] = 3;
                Alda_classificador[Ic] = 0;
            }else if(Alda_CBs){
                Alda_classificador[Ia] = 2;
                Alda_classificador[Ib] = 0;
                Alda_classificador[Ic] = 0;
            }else{
                Alda_classificador[Ia] = 1;
                Alda_classificador[Ib] = 0;
                Alda_classificador[Ic] = 0;
            }
        }
        if(Alda_sfftM[Ib]>=Alda_sfftM[Ia] && Alda_sfftM[Ib]>=Alda_sfftM[Ic]){
            aux = absvalue(Alda_faseM[Ia] - Alda_faseM[Ic]);
            if (aux > 180)  aux = 180 - (aux - 180);
            aux1 = absvalue(Alda_faseM[Ib] - Alda_faseM[Ia]);
            if (aux1 > 180) aux1 = 180 - (aux1 - 180);
            aux2 = absvalue(Alda_faseM[Ib] - Alda_faseM[Ic]);
            if (aux2 > 180) aux2 = 180 - (aux2 - 180);

            xua = absvalue(Alda_sfftM[Ia]-Alda_sfftM[Ic]);
            xua1 = absvalue(Alda_sfftM[Ib]-Alda_sfftM[Ia]);
            xua2 = absvalue(Alda_sfftM[Ib]-Alda_sfftM[Ic]);

            if(aux < aux1 && aux < aux2){
                if(aux1 > 90 && Alda_CBs){
                    Alda_classificador[Ia] = 0;
                    Alda_classificador[Ib] = 4;
                    Alda_classificador[Ic] = 0;
                }else if(Alda_CBs){
                    Alda_classificador[Ia] = 0;
                    Alda_classificador[Ib] = 2;
                    Alda_classificador[Ic] = 0;
                }else{
                    Alda_classificador[Ia] = 0;
                    Alda_classificador[Ib] = 1;
                    Alda_classificador[Ic] = 0;
                }
            }else if(aux1 < aux2 && aux1 < aux && (xua1 < xua2 && xua1 < xua)){
                Alda_classificador[Ia] = 0;
                Alda_classificador[Ib] = 0;
                Alda_classificador[Ic] = 3;
            }else if(aux2 < aux1 && aux2 < aux && (xua2 < xua1 && xua2 < xua)){
                Alda_classificador[Ia] = 3;
                Alda_classificador[Ib] = 0;
                Alda_classificador[Ic] = 0;
            }else if(Alda_CBs){
                Alda_classificador[Ia] = 0;
                Alda_classificador[Ib] = 2;
                Alda_classificador[Ic] = 0;
            }else{
                Alda_classificador[Ia] = 0;
                Alda_classificador[Ib] = 1;
                Alda_classificador[Ic] = 0;
            }
        }
        if(Alda_sfftM[Ic]>=Alda_sfftM[Ia] && Alda_sfftM[Ic]>=Alda_sfftM[Ib]){
            aux = absvalue(Alda_faseM[Ia] - Alda_faseM[Ib]);
            if (aux > 180)  aux = 180 - (aux - 180);
            aux1 = absvalue(Alda_faseM[Ic] - Alda_faseM[Ia]);
            if (aux1 > 180) aux1 = 180 - (aux1 - 180);
            aux2 = absvalue(Alda_faseM[Ic] - Alda_faseM[Ib]);
            if (aux2 > 180) aux2 = 180 - (aux2 - 180);

            xua = absvalue(Alda_sfftM[Ia]-Alda_sfftM[Ib]);
            xua1 = absvalue(Alda_sfftM[Ic]-Alda_sfftM[Ia]);
            xua2 = absvalue(Alda_sfftM[Ic]-Alda_sfftM[Ib]);

            if(aux < aux1 && aux < aux2){
                if(aux1 > 90 && Alda_CBs){
                    Alda_classificador[Ia] = 0;
                    Alda_classificador[Ib] = 0;
                    Alda_classificador[Ic] = 4;
                }else if(Alda_CBs){
                    Alda_classificador[Ia] = 0;
                    Alda_classificador[Ib] = 0;
                    Alda_classificador[Ic] = 2;
                }else{
                    Alda_classificador[Ia] = 0;
                    Alda_classificador[Ib] = 0;
                    Alda_classificador[Ic] = 1;
                }
            }else if(aux1 < aux2 && aux1 < aux && (xua1 < xua2 && xua1 < xua)){
                Alda_classificador[Ia] = 0;
                Alda_classificador[Ib] = 3;
                Alda_classificador[Ic] = 0;
            }else if(aux2 < aux1 && aux2 < aux && (xua2 < xua1 && xua2 < xua)){
                Alda_classificador[Ia] = 3;
                Alda_classificador[Ib] = 0;
                Alda_classificador[Ic] = 0;
            }else if(Alda_CBs){
                Alda_classificador[Ia] = 0;
                Alda_classificador[Ib] = 0;
                Alda_classificador[Ic] = 2;
            }else{
                Alda_classificador[Ia] = 0;
                Alda_classificador[Ib] = 0;
                Alda_classificador[Ic] = 1;
            }
        }
    }else{
        Alda_classificador[Ia] = 0;
        Alda_classificador[Ib] = 0;
        Alda_classificador[Ic] = 0;
    }

////****************************************************************************
terminaprocesso();
//caso estore o uso da CPU será incrementado o counter_process para resolver o problema
if(CPU_percent>=99){
  counter_proc++;
}
}
#endif
/*******************************************************************************
 *          DETECTOR DO SUBRAMAIN
 ******************************************************************************/
#if metodo==5
void subranain2012(void){
iniciaprocesso();
////****************************************************************************
//******************************************************************************
      counter_flag = 0;
      pv_hold = pv;

      float *Sub_temp;

      int k;
      int jj;
      int i;

      for(k=0;k<FFTLenWindow*amostragem;k++){

      if((pv_hold+k-FFTLenWindow*amostragem)<0){
          jj = pv_hold+k-FFTLenWindow*amostragem+FFTLenWindow*amostragem*Nciclos;
      }else{
          jj = pv_hold+k-FFTLenWindow*amostragem;
      }

      for(i=0;i<ChannelsRead;i++){
          DATA_VECTOR_HOLD[i][k] = DATA_V_I_BASE[i]*DATA_VECTOR[i][jj];
      }
      }

      Sub_divisor = Sub_nciclos*round(amostragem/counter_proc);

      fft_complete(1,1);

      fft_complete(6,1);
      fft_complete(6,3);

      phase[6][1] =phase_biuld(6,1,1);


      Sub_temp = subtraifasor(fftS[6][1],phase[6][1],Sub_fftS_media[6][1],Sub_fase_media[6][1]);

      Sub_modulo1harm = Sub_temp[1];
      Sub_angulo1harm = Sub_temp[0];


      phase[6][3] = phase_biuld(6,3,6);

      phase[6][2] = 3*(Sub_angulo1harm-phase[6][1]);

      phase[6][0] = phase[6][3] - phase[6][2];

      while(phase[6][0]>=180.0){
          phase[6][0] -= 360;
      }
      while(phase[6][0]<=-180.0){
          phase[6][0] += 360;
      }

      Sub_temp = subtraifasor(fftS[6][3], 0,Sub_fftS_media[6][3],0);

      Sub_modulo3harm = Sub_temp[1];
      Sub_angulo3harm = Sub_temp[0];

      if (Sub_firt_detection == false){
          Sub_temp = mediadafasor(fftS[6][1],phase[6][1],Sub_fftS_media[6][1],Sub_fase_media[6][1],Sub_divisor);

          Sub_fftS_media[6][1] = Sub_temp[1];
          Sub_fase_media[6][1] = Sub_temp[0];


          Sub_temp = mediadafasor(fftS[6][3],0,Sub_fftS_media[6][3],0,Sub_divisor);

          Sub_fftS_media[6][3] = Sub_temp[1];
          Sub_fase_media[6][3] = Sub_temp[0];
      }

/**
* Sub detector
*/




     if(Sub_modulo1harm>2) {
         Sub_relation = Sub_modulo3harm/Sub_modulo1harm;
     }else{
         Sub_relation = 0;
     }
     if(Sub_relation >= 0.02){
         if(teste_counter>contadordeteste){
             teste_counter=0;
         }else{
             teste_counter++;
         }
         if(phase[6][0]<=-120||phase[6][0] >= 120){
             Sub_firt_detection = true;
         }else{
             Sub_firt_detection = false;
             Sub_FAI_detect_counter = 0;
         }
     }else{
         Sub_firt_detection = false;
         Sub_FAI_detect_counter = 0;
     }
     if(Sub_FAI_detect_counter >= Sub_time){
         Sub_FAI_detect = true;
         FAI_detect = true;
         Sub_FAI_detect_counter = Sub_time;
     }else{
         Sub_FAI_detect = false;
     }

terminaprocesso();
//caso estore o uso da CPU será incrementado o counter_process para resolver o problema
      if(CPU_percent>=99){
          counter_proc++;
      }
//      GpioDataRegs.GPCCLEAR.bit.GPIO92 = 1;
}
#endif
/*******************************************************************************
 *          DETECTOR DA ERICA
 ******************************************************************************/
#if metodo==1
/******************************************************************************
 *          DETECÇAO DE FAI VIA HARMONICOS da Erica
 ******************************************************************************/
int Er_harmonic_detect(int fase){

    Er_relation3[fase] = fftS[fase+3][3]/fftS[fase+3][1];

    if(Er_relation3[fase] >= pickup3 ){
        Er_harm_first_detection[fase] = true;
        if(Er_harm_time_detection[fase]>Er_time){
            Er_relation2[fase] = fftS[fase+3][2]/fftS[fase+3][1];
            Er_relation5[fase] = fftS[fase+3][5]/fftS[fase+3][1];
            if((Er_relation2[fase] >= pickup2)&&(Er_relation5[fase] >= pickup5)){
                Er_harm_detected[fase] = 1;
                return 1;
            }
        }
    }else{
        Er_harm_detected[fase] = 0;
        Er_harm_first_detection[fase] = 0;
        Er_harm_time_detection[fase] = 0;
    }

    if((Er_relation3[fase] >= pickup3*0.2) && (Er_fase_media[fase+3][3] >= 120 || Er_fase_media[fase+3][3] <= -150)){
        Er_first_fase_detection[fase] = true;
        if(Er_time_fase_detection[fase]>Er_time){
            Er_fase_detected[fase] = 1;
            return 1;
        }
    }else{
        Er_fase_detected[fase] = 0;
        Er_first_fase_detection[fase] = 0;
        Er_time_fase_detection[fase] = 0;
    }

return 0;
}

void erica2018(void){
iniciaprocesso();
////****************************************************************************
//******************************************************************************
      counter_flag = 0;
      pv_hold = pv;

      float *Er_temp;

      int k;
      int jj;
      int i;

      for(k=0;k<FFTLenWindow*amostragem;k++){

      if((pv_hold+k-FFTLenWindow*amostragem)<0){
          jj = pv_hold+k-FFTLenWindow*amostragem+FFTLenWindow*amostragem*Nciclos;
      }else{
          jj = pv_hold+k-FFTLenWindow*amostragem;
      }

      for(i=0;i<ChannelsRead;i++){
          DATA_VECTOR_HOLD[i][k] = DATA_V_I_BASE[i]*DATA_VECTOR[i][jj];
      }
      }


      fft_complete(VA,1);
      fft_complete(VB,1);
      fft_complete(VC,1);

      fft_complete(IA,1);
      fft_complete(IB,1);
      fft_complete(IC,1);

      fft_complete(IA,2);
      fft_complete(IA,3);
      fft_complete(IA,5);

      fft_complete(IB,2);
      fft_complete(IB,3);
      fft_complete(IB,5);

      fft_complete(IC,2);
      fft_complete(IC,3);
      fft_complete(IC,5);

      phase[IA][3] = phase_biuld(IA,3,VA);
      phase[IB][3] = phase_biuld(IB,3,VB);
      phase[IC][3] = phase_biuld(IC,3,VC);

      Er_divisor = Er_nciclos*round(amostragem/counter_proc);
 //     if(Er_firt_detection==false){

          Er_temp = mediadafasor(fftS[3][3],phase[3][3],Er_fftS_media[3][3],Er_fase_media[3][3],Er_divisor);

          Er_fftS_media[3][3] = Er_temp[1];
          Er_fase_media[3][3] = Er_temp[0];

          Er_temp = mediadafasor(fftS[4][3],phase[4][3],Er_fftS_media[4][3],Er_fase_media[4][3],Er_divisor);

          Er_fftS_media[4][3] = Er_temp[1];
          Er_fase_media[4][3] = Er_temp[0];

          Er_temp = mediadafasor(fftS[5][3],phase[5][3],Er_fftS_media[5][3],Er_fase_media[5][3],Er_divisor);

          Er_fftS_media[5][3] = Er_temp[1];
          Er_fase_media[5][3] = Er_temp[0];

/**
 * Erica detector
 */

    if( ( fftS[IA][1] > 5) && Er_harmonic_detect(Ia)){
        Er_FAI_detect[Ia] = true;
        FAI_detect = true;
        FAI_fase_detect[0] = 1; 
    }else{
    	Er_FAI_detect[Ia] = false;
        Er_harm_detected[Ia] = 0;
        Er_fase_detected[Ia] = 0;
    }
    if( ( fftS[IB][1] > 5) && Er_harmonic_detect(Ib)){
        Er_FAI_detect[Ib] = true;
        FAI_detect = true;
        FAI_fase_detect[1] = 1; 
    }else{
    	Er_FAI_detect[Ib] = false;
        Er_harm_detected[Ib] = 0;
        Er_fase_detected[Ib] = 0;
    }
    if( ( fftS[IC][1] > 5) && Er_harmonic_detect(Ic)){
        Er_FAI_detect[Ic] = true;
        FAI_detect = true;
        FAI_fase_detect[2] = 1; 
    }else{
    	Er_FAI_detect[Ic] = false;
        Er_harm_detected[Ic] = 0;
        Er_fase_detected[Ic] = 0;
    }


terminaprocesso();
//caso estore o uso da CPU será incrementado o counter_process para resolver o problema
        if(CPU_percent>=99){
            counter_proc++;
        }
//      GpioDataRegs.GPCCLEAR.bit.GPIO92 = 1;
}
#endif
/*******************************************************************************
 *          DETECTOR DO TORRES
 ******************************************************************************/
#if metodo==0
void torres2014(void){
iniciaprocesso();
////****************************************************************************
//******************************************************************************
	
    float thd_aux;

      counter_flag = 0;
      pv_hold = pv;

      int k;
      int jj;
      int i;

      for(k=0;k<FFTLenWindow*amostragem;k++){

      if((pv_hold+k-FFTLenWindow*amostragem)<0){
          jj = pv_hold+k-FFTLenWindow*amostragem+FFTLenWindow*amostragem*Nciclos;
      }else{
          jj = pv_hold+k-FFTLenWindow*amostragem;
      }

      for(i=0;i<ChannelsRead;i++){
          DATA_VECTOR_HOLD[i][k] = DATA_V_I_BASE[i]*DATA_VECTOR[i][jj];
      }
      }
      
      fft_complete(VA,1);

      for(j=1;j<HarmNum;j++){
          fft_complete(IA,j);
          fft_complete(IB,j);
          fft_complete(IC,j);
      }
//      for(j=1;j<HarmNum;j++){
//		phase[IA][j] = phase_biuld(IA,j,VA);
//		phase[IB][j] = phase_biuld(IB,j,VA);
//		phase[IC][j] = phase_biuld(IC,j,VA);
//	  }


    thd_aux = fftS[IA][2]*fftS[IA][2]+fftS[IA][3]*fftS[IA][3]+fftS[IA][4]*fftS[IA][4]+fftS[IA][5]*fftS[IA][5]+fftS[IA][6]*fftS[IA][6]+fftS[IA][7]*fftS[IA][7]+fftS[IA][8]*fftS[IA][8]+fftS[IA][9]*fftS[IA][9];

    torres_THD[Ia] = sqrt(thd_aux)/sqrt(thd_aux+fftS[IA][1]*fftS[IA][1]);

    thd_aux = fftS[IB][2]*fftS[IB][2]+fftS[IB][3]*fftS[IB][3]+fftS[IB][4]*fftS[IB][4]+fftS[IB][5]*fftS[IB][5]+fftS[IB][6]*fftS[IB][6]+fftS[IB][7]*fftS[IB][7]+fftS[IB][8]*fftS[IB][8]+fftS[IB][9]*fftS[IB][9];

    torres_THD[Ib] = sqrt(thd_aux)/sqrt(thd_aux+fftS[IB][1]*fftS[IB][1]);

    thd_aux = fftS[IC][2]*fftS[IC][2]+fftS[IC][3]*fftS[IC][3]+fftS[IC][4]*fftS[IC][4]+fftS[IC][5]*fftS[IC][5]+fftS[IC][6]*fftS[IC][6]+fftS[IC][7]*fftS[IC][7]+fftS[IC][8]*fftS[IC][8]+fftS[IC][9]*fftS[IC][9];

    torres_THD[Ic] = sqrt(thd_aux)/sqrt(thd_aux+fftS[IC][1]*fftS[IC][1]);

//passar tudo pra retangular pra somar o fasor

	torres_oddharm[Ia] = fftS[IA][2] + fftS[IA][4] + fftS[IA][6] + fftS[IA][8];
    torres_oddharm[Ib] = fftS[IB][2] + fftS[IB][4] + fftS[IB][6] + fftS[IB][8];
    torres_oddharm[Ic] = fftS[IC][2] + fftS[IC][4] + fftS[IC][6] + fftS[IC][8];

    torres_evenharm[Ia] = fftS[IA][3] + fftS[IA][5] + fftS[IA][7] + fftS[IA][9];
    torres_evenharm[Ib] = fftS[IB][3] + fftS[IB][5] + fftS[IB][7] + fftS[IB][9];
    torres_evenharm[Ic] = fftS[IC][3] + fftS[IC][5] + fftS[IC][7] + fftS[IC][9];

    if(fftS[IA][3] > torres_oddharm[Ia]){
    	torres_first_detection[Ia] = true;
    	if(torres_time_detection[Ia] >= torres_delay){
    		if(torres_THD[Ia] > torres_k*(torres_THD[Ia]+torres_THD[Ib]+torres_THD[Ic])/3){
    			FAI_detect = true;
    			torres_FAI_detect[Ia] = true;
    		}else{
    			torres_FAI_detect[Ia] = false;
    		}
    		torres_time_detection[Ia] = 200;
    	}else{
    		torres_FAI_detect[Ia] = false;
    	}
    }else{
    	torres_first_detection[Ia] = false;
    	torres_time_detection[Ia] = 0;
    	torres_FAI_detect[Ia] = false;
    }
    if(fftS[IB][3] > torres_oddharm[Ib]){
    	torres_first_detection[Ib] = true;
    	if(torres_time_detection[Ib] >= torres_delay){
    		if(torres_THD[Ib] > torres_k*(torres_THD[Ia]+torres_THD[Ib]+torres_THD[Ic])/3){
    			FAI_detect = true;
    			torres_FAI_detect[Ib] = true;
    		}else{
    			torres_FAI_detect[Ib] = false;
    		}
    		torres_time_detection[Ib] = 200;
    	}else{
    		torres_FAI_detect[Ib] = false;
    	}
    }else{
    	torres_first_detection[Ib] = false;
    	torres_time_detection[Ib] = 0;
    	torres_FAI_detect[Ib] = false;
    }
    if(fftS[IC][3] > torres_oddharm[Ic]){
    	torres_first_detection[Ic] = true;
    	if(torres_time_detection[Ic] >= torres_delay){
    	    if(torres_THD[Ic] > torres_k*(torres_THD[Ia]+torres_THD[Ib]+torres_THD[Ic])/3){
    			FAI_detect = true;
    			torres_FAI_detect[Ic] = true;
    		}else{
    			torres_FAI_detect[Ic] = false;
    		}
    		torres_time_detection[Ic] = 200;
    	}else{
    		torres_FAI_detect[Ic] = false;
    	}
    }else{
    	torres_first_detection[Ic] = false;
    	torres_time_detection[Ic] = 0;
    	torres_FAI_detect[Ic] = false;
    }
// fazer o calculo da distorcao harmonicas aki e criar uma flag que fica olhando se THD > kTHD medio das tres fases


terminaprocesso();
//caso estore o uso da CPU será incrementado o counter_process para resolver o problema
        if(CPU_percent>=99){
            counter_proc++;
        }
//      GpioDataRegs.GPCCLEAR.bit.GPIO92 = 1;
}
#endif
/*******************************************************************************
 *       INTERPRETADOR DE COMANDOS DO SISTEMAL
 *       todos comando recebidos via terminal Serial
 ******************************************************************************/
void interpretador(void){
    int retorno;

    retorno = strncmp(msgR,palavra[0],9);
    if(retorno == 0){
        send_data = 0;
        msgS = "ADC iniciado\n";
        amostragem_flag = 1;
        goto end;
    }

    retorno = strncmp(msgR,palavra[1],8);
    if(retorno == 0){
        send_data = 0;
        msgS = "ADC parado\n";
        amostragem_flag = 0;
        goto end;
    }
    retorno = strncmp(msgR,palavra[2],8);
    if(retorno == 0){
        send_data = 1;
        amostragem_flag = 0;
        point_pv = pv+1;
        send_n_channels = ChannelsRead;
        send_ch_pv = 0;
        msgS = "Obtendo todos os dados... \n";
        datalen.vector_send = &DATA_VECTOR_EX[0][0];
        datalen.len = FFTLenWindow*amostragem*Nciclos;
        goto end;
    }

    retorno = strncmp(msgR,palavra[3],9);
    if(retorno == 0){
        msgS = "FAI reset\n";
        FAI_detect = false;
        FAI_fase_detect[0] = 0;
        FAI_fase_detect[1] = 0;
        FAI_fase_detect[2] = 0;
        Event_report = false;
        event_enable = false;
        send_data = 0;
        enable_send_FAI = 1;
        send_mode = 0;
        amostragem_flag = 1;
        goto end;
    }

    retorno = strncmp(msgR,palavra[4],12);
    if(retorno == 0){
        Event_report = false;
        FAI_detect = false;
        FAI_fase_detect[0] = 0;
        FAI_fase_detect[1] = 0;
        FAI_fase_detect[2] = 0;
        event_enable = true;
        msgS = "Event report\n";
        send_data = 0;
        enable_send_FAI = 1;
        send_mode = 0;
        amostragem_flag = 1;
        goto end;
    }

    msgS = "Comand not vailed\n";

end:
    ch_pvS = 0;
    SciaRegs.SCICTL2.bit.TXINTENA = 1;
    SciaRegs.SCITXBUF.all = (Uint16)msgS[ch_pvS];
}
/*******************************************************************************
 * ******************** fim dos metodos baseados em harmonicos *****************
 * ****************************************************************************/
/*******************************************************************************
 *
 ******************************************************************************/
void Conversation_Done(void){
/***        SALVO OS DADOS     */

    DATA_VECTOR_EX[0][pv] =  (int16)(100*Alda_sfftM[Ia]);
    DATA_VECTOR_EX[1][pv] =  (int16)(100*Alda_sfftM[Ib]);
    DATA_VECTOR_EX[2][pv] =  (int16)(100*Alda_sfftM[Ic]);
    DATA_VECTOR_EX[3][pv] =  (int16)(100*Alda_faseM[Ia]);
    DATA_VECTOR_EX[4][pv] =  (int16)(100*Alda_faseM[Ib]);
    DATA_VECTOR_EX[5][pv] =  (int16)(100*Alda_faseM[Ic]);
    DATA_VECTOR_EX[6][pv] =  (int16)(Alda_classificador[Ia]);
    DATA_VECTOR_EX[7][pv] =  (int16)(Alda_classificador[Ib]);
    DATA_VECTOR_EX[8][pv] =  (int16)(Alda_classificador[Ic]);

    int j;
      pv_ant = pv;
      if(pv==lenght_pv-1){
         pv=0;
      }else{
         pv++;
      }

    switch(imputmode){
        case 0:
            DATA_VECTOR[0][pv] =  (DATA[4]);
    #if ChannelsRead>1
            DATA_VECTOR[1][pv] =  (DATA[5]);
    #endif
    #if ChannelsRead>2
            DATA_VECTOR[2][pv] =  (DATA[6]);
    #endif
    #if ChannelsRead>3
            DATA_VECTOR[3][pv] =  (DATA[1]);
    #endif
    #if ChannelsRead>4
            DATA_VECTOR[4][pv] =  (DATA[2]);
            DATA_VECTOR[5][pv] =  (DATA[3]);
            DATA_VECTOR[6][pv] =  (DATA[0]);
            //DATA_VECTOR[7][pv] =  (DATA[7]);
    #endif
    #if ChannelsRead>8
            DATA_VECTOR[8][pv] =  (DATA[14]);
            DATA_VECTOR[9][pv] =  (DATA[13]);
            DATA_VECTOR[10][pv] =  (DATA[12]);
            DATA_VECTOR[11][pv] =  (DATA[15]);
    #endif
    #if ChannelsRead>12
            DATA_VECTOR[12][pv] =  (DATA[11]);
            DATA_VECTOR[13][pv] =  (DATA[10]);
            DATA_VECTOR[14][pv] =   (DATA[9]);
            DATA_VECTOR[15][pv] =   (DATA[8]);
    #endif
        break;
        case 1:
            for (j=0;j<ChannelsRead-1;j++){
            DATA_VECTOR[j][pv] =  (DATA[j]);
            }
        break;
    }

    if(DATA[ChannelsRead-1] >= 5000 || DATA[ChannelsRead-1] <= -5000) Event_report = true;

 #if metodo!=3
    if(FAI_detect) DATA_VECTOR[ChannelsRead-1][pv] = 0x0FFF;
    else if (Event_report) DATA_VECTOR[ChannelsRead-1][pv] = 0x04FF;
    else  DATA_VECTOR[ChannelsRead-1][pv] = 0x0000;
 #endif

////////////////////////////////////  Selecionar fase para calculo da frequencia
     time_cicle++;
     if((DATA_VECTOR[fase_REF][pv_ant]<0)&&(DATA_VECTOR[fase_REF][pv]>=0)){
        flag_zera_tempo = 1;
        time_ruy_anterior = time_ruy;
        time_ruy = 0.01*timerum_t_us*DATA_VECTOR[fase_REF][pv]/(DATA_VECTOR[fase_REF][pv]-DATA_VECTOR[fase_REF][pv_ant]);
        timer_ajust += 0.01*time_cicle*timerum_t_us - time_ruy + time_ruy_anterior;
        time_cicle = 0;
        count_zerocross_timer++;
        if(count_zerocross_timer==10){
           frequencia = 10000000.0/timer_ajust;
           timerum_t_us_new = (Uint16)round(10.0*timer_ajust/(amostragem));
           if((timerum_t_us_new != timerum_t_us) && (fftS[fase_REF][1] > 10) && (70>(frequencia)&&(frequencia)>40)){
              Update_timerum = 1;
           }
              count_zerocross_timer=0;
              timer_ajust = 0; 
        }

     }
     counter_flag++;
}

/** Decodificador de numeros inteiros em char que serao enviados via serial
 *
 */
char * decoder_intchar(int32 send_value){
    int res;
    static char len_data[10];
    int i = 1;
    int j = 1;
    char valchar_aux[10];
    len_data[0] = 0;
    while(send_value>=10){
        res = send_value % 10;
        valchar_aux[i] = (char)(res+48);
        send_value = send_value / 10;
        i++;
    }
    res = send_value % 10;
    valchar_aux[i] = (char)(res+48);
    while(i!=0){
        len_data[j] = valchar_aux[i];
        j++;
        i--;
    }
    len_data[j] = 10;
    return len_data;
}
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *              ALL INTERRUPS
 ******************************************************************************/
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *              INTERRUP TO SERIAL RECIVER
 ******************************************************************************/

interrupt void sciaRxFifoIsr(void){

    Event_report = false;
    FAI_detect = false;

    char dataSCIrecive;  //Char utilizado para receber STRINGS
    dataSCIrecive = SciaRegs.SCIRXBUF.all;  // Read data
    msgR[ch_pvR] = dataSCIrecive; //Salvo caractere recebido no vetor principal
    ch_pvR++; // incremento a posição do vetor para o proximo caractere
    if(ch_pvR >= 30){   // tamanho maximo de msg estabelecido
        msgS = "too many words\n";
        ch_pvS = 0;
        ch_pvR = 0;
        dataSCIrecive = '0';
        PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
        SciaRegs.SCICTL2.bit.TXINTENA = 1;
        SciaRegs.SCITXBUF.all = (Uint16)msgS[ch_pvS];
    }
    if(dataSCIrecive == 10){//se o caracter recebido for "\n" então é fim de msg
        last_ch_pvR = ch_pvR-1;
        ch_pvR = 0;
        interpretador();
    }
    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
}

////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *               INTERRUP TO SERIAL SEND
 ******************************************************************************/

interrupt void sciaTxFifoIsr(void){
    int16 valor;
    int32 aux;
switch (send_mode){ // mode 0 é caractere e 1 é inteiro
case 0: // envia caracteres quando send_mode = 0
    ch_pvS++;  // incrementa a posição da string a ser enviada
    SciaRegs.SCITXBUF.all = (Uint16)msgS[ch_pvS];  // escreve no registrador de envio
    if(msgS[ch_pvS] == 10){  // verifica fim de mensagem "\n"
        switch (send_data){  // controle de protocolo.
        case 0:  // termina a transmissão. envia somente caracter
            SciaRegs.SCICTL2.bit.TXINTENA = 0;
        break;
        case 1:  // envia o comprimento dos vetores via char
            msgS = decoder_intchar(datalen.len);
            ch_pvS = 0;
            send_data = 2;
        break;
        case 2:
            msgS = decoder_intchar(RANGE);
            ch_pvS = 0;
            send_data = 3;
        break;
        case 3:  // envia o numero de canais via char
            msgS = decoder_intchar(send_n_channels);
            ch_pvS = 0;
            send_data = 4;
        break;
        case 4: //envia RTC e RTP completo
            aux = (int32)(DATA_V_I_RTC_RTP_EX[pv_tctp]);
            msgS = decoder_intchar(aux);
            if (pv_tctp >= 15) {
                pv_tctp = 0;
                send_data = 5;
                ch_pvS = 0;
            }else{
                pv_tctp++;
                ch_pvS = 0;
            }
        break;
        case 5: //envia RTC e RTP completo
            aux = (int32)(ATP_ajuste[pv_tctp]*1000);
            msgS = decoder_intchar(aux);
            if (pv_tctp >= 15) {
                pv_tctp = 0;
                send_data = 6;
                ch_pvS = 0;
            }else{
                pv_tctp++;
                ch_pvS = 0;
            }
        break;
        case 6:
            ch_pvS = 0;
            send_data = 0;
            send_mode = 1;
        break;
        default:
            SciaRegs.SCICTL2.bit.TXINTENA = 0;
        break;
        }
    }
break;
case 1: // envia dados
    if(ch_pvS >= datalen.len){
        if(send_n_channels>1){
            ch_pvS = 0;
            send_n_channels--;
            send_ch_pv++;
            datalen.vector_send = &DATA_VECTOR_EX[send_ch_pv][0];
            goto jump;
        }else{
            msgS = "\n";
            ch_pvS = 0;
            send_mode = 0;
            amostragem_flag = 1;
            SciaRegs.SCITXBUF.all = (Uint16)msgS[ch_pvS];
        }
    }else{
jump:
        if((point_pv+ch_pvS)>=datalen.len){
            valor = (Uint16)*(datalen.vector_send+point_pv+ch_pvS-datalen.len);
        }else{
            valor = (Uint16)*(datalen.vector_send+point_pv+ch_pvS);
        }
        switch (send_position){
        case 0:
            SciaRegs.SCITXBUF.all = (valor&0xFF00)>>8;
            send_position = 1;
        break;
        case 1:
            SciaRegs.SCITXBUF.all = valor&0x00FF;
            send_position = 0;
            ch_pvS++;
        break;
        }
    }
break;
}
    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;   // Clear SCI Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
}
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *              EXTERNAL INTERRUPT FROM BUSSY
 *  Estas interrupções são recebidas de um pino externo que vem do AD
 *  cada AD gara um interrupção quando está pronta a conversão analogica
 *  Apos estar pronto as conversões, estas interrupções abilitam os
 *  blocos Mcbsp para iniciarem as transmissões dos dados.
 *  A transmissão é feita por 2 blocos Mcbsp e 2 SPI, cada bloco recebe 4 bloco
 *  de dados. Esses dados são montados em DATA.
 *  A cada bloco de dados recebidos é chamada a interrupção dos Mcbsp.
 ******************************************************************************/
//
// xint1_isr - External Interrupt 1 ISR
//
__interrupt void xint1_isr(void){
    transmit_a = 0;
McbspaRegs.SPCR2.bit.FRST = 1;
McbspaRegs.SRGR2.bit.FSGM = 1;
PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
//
// xint2_isr - External Interrupt 2 ISR
//
__interrupt void xint2_isr(void){
    transmit_b = 8;
McbspbRegs.SPCR2.bit.FRST = 1;
McbspbRegs.SRGR2.bit.FSGM = 1;
PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *            Mcbsp_RxINTA_ISR - Mcbsp Receive ISR
 *  Cada interrupção traz 2 dados, um do Mcbsp e um do SPI, esses dados são
 *  salvos em DATA. Cada amostragem irá chamar 4 vezes esta interupção para [
 *  completa de 8 dados aqui e mais 8 na interrupção de baixo.
 ******************************************************************************/
__interrupt void Mcbsp_RxINTA_ISR(void){
//    if(transmit_a == 4) transmit_a = 4;
    DATA[transmit_a] = McbspaRegs.DRR1.all;  // salva os dados do Mcbsp
       DATA[transmit_a+4] = SpiaRegs.SPIRXBUF;  // salva os dados do SPI
   if(transmit_a>=3){
    McbspaRegs.SRGR2.bit.FSGM = 0;
    McbspaRegs.SPCR2.bit.FRST = 0;
   }
   if(transmit_a==3){
    GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
    Conversion_Done_a = 1;
    if(Conversion_Done_b==1){
      Conversion_Done_a = 0;
      Conversion_Done_b = 0;
      Conversation_Done();
    }
   }
   transmit_a++;
PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *               Mcbsp_RxINTB_ISR - Mcbsp Receive ISR
 *
 ******************************************************************************/
__interrupt void Mcbsp_RxINTB_ISR(void){
//     if(transmit_b == 12) transmit_b = 12;
    DATA[transmit_b] = McbspbRegs.DRR1.all;
    DATA[transmit_b+4] = SpibRegs.SPIRXBUF;
  if(transmit_b>=11){
    McbspbRegs.SRGR2.bit.FSGM = 0;
    McbspbRegs.SPCR2.bit.FRST = 0;
  }
  if(transmit_b==11){
      GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;
      GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
      Conversion_Done_b = 1;
      if(Conversion_Done_a==1){
        Conversion_Done_b = 0;
        Conversion_Done_a = 0;
        Conversation_Done();
      }
  }
  transmit_b++;
PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *                            TIMER AUXILIAR
 ******************************************************************************/

__interrupt void cpu_timer0_isr(void){
// Acknowledge this interrupt to receive more interrupts from group 1
EALLOW;
CpuTimer0Regs.TIM.all = 0;
EDIS;
PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}

////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *                         TIMER GLOBAL RELOGIO
 ******************************************************************************/

__interrupt void cpu_timer2_isr(void){
///////////////////   milisecunds counter++
    global_timer++;
/////////////////////////////////////////////////////////////////////////////////////
#if metodo==0
    if(torres_first_detection[Ia]) torres_time_detection[Ia]++;
    if(torres_first_detection[Ib]) torres_time_detection[Ib]++;
    if(torres_first_detection[Ic]) torres_time_detection[Ic]++;
#endif
/////////////////////////////////////////////////////////////////////////////////////
#if metodo==1 
    if(Er_harm_first_detection[Ia]) Er_harm_time_detection[Ia]++;
    if(Er_harm_first_detection[Ib]) Er_harm_time_detection[Ib]++;
    if(Er_harm_first_detection[Ic]) Er_harm_time_detection[Ic]++;
 
    if(Er_first_fase_detection[Ia]) Er_time_fase_detection[Ia]++;
    if(Er_first_fase_detection[Ib]) Er_time_fase_detection[Ib]++;
    if(Er_first_fase_detection[Ic]) Er_time_fase_detection[Ic]++;
#endif
/////////////////////////////////////////////////////////////////////////////////////
#if metodo==3
    if(Br_inicialset[Va]) Br_time[Va]++;
    if(Br_inicialset[Vb]) Br_time[Vb]++;
    if(Br_inicialset[Vc]) Br_time[Vc]++;
#endif
/////////////////////////////////////////////////////////////////////////////////////
#if metodo==5
    if(Sub_firt_detection) Sub_FAI_detect_counter++;
#endif
/////////////////////////////////////////////////////////////////////////////////////
#if metodo==4
    if(Ka_flag_first_detection==1) Ka_counter_16ms++;
    if(Ka_second_detection==1) Ka_counter_1s++;
#endif
/////////////////////////////////////////////////////////////////////////////////////
#if metodo==2
    if(Alda_firt_detection){
        if(Alda_detection) Alda_FAI_detect_counter++;
        else Alda_FAI_detect_counter--;
    }else{
        if(Alda_CBs){
            Alda_reset_CB++;
            if(Alda_reset_CB>5000){
                Alda_CBs=0;
                Alda_CBsf[0]=0;
                Alda_CBsf[1]=0;
                Alda_CBsf[2]=0;
                Alda_reset_CB=0;
            }
        }
    }
    if(Alda_inicial_set){
        Alda_inicial_counter++;
    }
#endif


if (Event_report){
    if(FAI_detect==0) TIME_DETECTION++;
}else{
    TIME_DETECTION = 0;
}

if (FAI_detect) GpioDataRegs.GPCSET.bit.GPIO92 = 1; //TESTE
else GpioDataRegs.GPCCLEAR.bit.GPIO92 = 1; //TESTE

    if(FAI_detect || Event_report&&event_enable){
        time_FAI++;
    }else{
        time_FAI=0;
    }

    if(enable_send_FAI && SciaRegs.SCICTL2.bit.TXINTENA == 0 && ( (time_FAI>150)&&!event_enable || (time_FAI>500)&&event_enable) ){
        send_data = 1;
        enable_send_FAI = 0;
        amostragem_flag = 0;
        send_n_channels = ChannelsRead;
        send_ch_pv = 0;
        msgS = "FAI detectada. Obtendo todos os dados... \n";
        datalen.vector_send = &DATA_VECTOR[0][0];
        datalen.len = FFTLenWindow*amostragem*Nciclos;
        time_FAI = 0;
//        FAI_detect = false;
//        Event_report = false;
//        event_enable = false;
        ch_pvS = 0;
        point_pv = pv+1;
        SciaRegs.SCICTL2.bit.TXINTENA = 1;
        SciaRegs.SCITXBUF.all = (Uint16)msgS[ch_pvS];
    }

/////////////////////////////////////////////////////////////////////////////////////
    if(amostragem_flag){
        time_led++;
        if (time_led>=500){
            GpioDataRegs.GPBTOGGLE.bit.GPIO42 = 1; //LED 1
            GpioDataRegs.GPBTOGGLE.bit.GPIO43 = 1; //LED 2
            time_led = 0;
        }
    }else if (SciaRegs.SCICTL2.bit.TXINTENA){
        time_led++;
        if (time_led>=50){
            GpioDataRegs.GPBTOGGLE.bit.GPIO42 = 1; //LED 1
            GpioDataRegs.GPBTOGGLE.bit.GPIO43 = 1; //LED 2
            time_led = 0;
        }
    }else{
        GpioDataRegs.GPBSET.bit.GPIO42 = 1; //LED 1
        GpioDataRegs.GPBSET.bit.GPIO43 = 1; //LED 2
        time_led = 0;
    }
///////////////////   secounds counter++
    if(global_timer>=1000){
        global_timer=0;
#if metodo==3
        prave = 0;
#endif
        global_seconds++;
///////////////////   minutes counter++
        if(global_seconds>=60){
            global_seconds=0;
            global_minutes++;
///////////////////   hours counter++
            if(global_minutes>=60){
                global_minutes=0;
                global_hours++;
                if(global_hours>=24){
                    global_hours=0;
                }
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 *                         TIMER DE AMOSTRAGEM
 ******************************************************************************/

__interrupt void cpu_timer1_isr(void){
DINT;
  if(amostragem_flag){
//  GpioDataRegs.GPASET.bit = GPIO10 | GPIO11;
  GpioDataRegs.GPASET.bit.GPIO10 = 1;   // irá chamar AD1 no pino externo
  GpioDataRegs.GPASET.bit.GPIO11 = 1;   // irá chamar AD2 no pino externo
  }
    if(Update_timerum&&enable_update){
        timerum_t_us = timerum_t_us_new;
        ConfigCpuTimer(&CpuTimer1, 2, timerum_t_us);
        CpuTimer1Regs.TCR.all = 0x4000;
        Update_timerum = 0;
    }
    timercontrol += 0.01*timerum_t_us;
EINT;
}

////////////////////////////////////////////////////////////////////////////////
////   MAIN     MAIN     MAIN     MAIN     MAIN     MAIN     MAIN     //////////
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * Função principal
 * Executa apenas uma vez
 * Inicializa os dispositivos e deixa pronto para funcionar
 */
void main(void){
/** Initialize System Control:
 *  PLL, WatchDog, enable Peripheral Clocks
 */
   InitSysCtrl();
/** Inicializo as funções de controle de hardware do Mcbsp, para maiores
 * informações consultar F2837xS_Mcbsp.c.
 */
   InitMcbspa();
   InitMcbspb();
   InitMcbspaGpio();
   InitMcbspbGpio();
   InitMcbspaInt();
   InitMcbspbInt();
/** Inicializo as GPIO do SPI
 *
 */
   InitSpiaGpio();
/** Clear all interrupts:
 *
 */
DINT;
IER = 0x0000;
IFR = 0x0000;
/** Initialize PIE control registers to their default state.
 *  The default state is all PIE __interrupts disabled and flags are cleared.
 */
   InitPieCtrl();
/** Initialize the PIE vector table with pointers to the shell Interrupt
 *  Service Routines (ISR).
 *  This will populate the entire table, even if the __interrupt
 *  is not used in this example.  This is useful for debug purposes.
 * The shell ISR routines are found in F2837xS_DefaultIsr.c.
 */
   InitPieVectTable();
/** Configura algumas funções de hardware
 *
 */
EALLOW;
ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0;   // for high-speed clock inside devices
ClkCfgRegs.SYSCLKDIVSEL.all = 0;       // disable divided clock system
/** Aponta os ponteiros das interrupções
 *
 */
PieVectTable.MCBSPA_RX_INT = &Mcbsp_RxINTA_ISR;
PieVectTable.MCBSPB_RX_INT = &Mcbsp_RxINTB_ISR;

PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;
PieVectTable.SCIA_TX_INT = &sciaTxFifoIsr;

PieVectTable.XINT1_INT = &xint1_isr;
PieVectTable.XINT2_INT = &xint2_isr;
EDIS;
/** Initialize the PIE vector table with pointers to the shell Interrupt
 *
 */
PieCtrlRegs.PIECTRL.bit.ENPIE =1;
PieCtrlRegs.PIEIER6.bit.INTx5 = 1;   // Enable PIE Group 6, INT 5
PieCtrlRegs.PIEIER6.bit.INTx7 = 1;   // Enable PIE Group 6, INT 7
PieCtrlRegs.PIEIER1.bit.INTx4 = 1;   // Enable PIE Group 1 INT4
PieCtrlRegs.PIEIER1.bit.INTx5 = 1;   // Enable PIE Group 1 INT5
PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE Group 9, INT1
PieCtrlRegs.PIEIER9.bit.INTx2 = 1;   // PIE Group 9, INT2
/** Initialize the Device Peripherals:
 *
 */
   Gpioconfig();
   external_interrupt();
   spi_fifo_init();     // Initialize the SPI FIFO
   mcbsp_a_init_dlb();  // Configura McbspA
   mcbsp_b_init_dlb();  // Configura McbspB
//   configureDAC(1);   // Configura o DAC
//   configureDAC(2);   // Configura o DAC
//   configureDAC(3);   // Configura o DAC

   timerum_t_us = (Uint16)round(100000000.0/(feqRede*amostragem)); //valor do timerum

   timer_config(timerzero_t_us, timerum_t_us, timerdois_t_us);
   scia_init();

////////////////////// Calculated Parameters //////////////////////////
   msgR = (char*)malloc(sizeof(char)*30); // aloca um espaço reservado
   msgS = (char*)malloc(sizeof(char)*30); // aloca um espaço reservado

   constant_1 = 2.0*FFTLenWindow*3.14159265359/(FFTLenWindow*amostragem);
   constant_dois = 2.0/(amostragem*FFTLenWindow);

   for(j=0;j<HarmNum;j++){
   for(k=0;k<FFTLenWindow*amostragem;k++){
     float  angle = constant_1*j*k;
       Cfc[j][k] = cosf(angle);
       Cfs[j][k] = -sinf(angle);
   }
   }

   for(k=0;k<ChannelsRead;k++){
#if ATP_NORMAL
       DATA_V_I_BASE[k]=DATA_V_I_BASE[k]*DATA_V_I_RTC_RTP[k]/ATP_ajuste[k];
#else
       DATA_V_I_BASE[k]=DATA_V_I_BASE[k]*DATA_V_I_RTC_RTP[k];
#endif
   }

#if metodo==3
   for(k=0;k<MMSELEN;k++){
       Br_SE[k] = 0.01;
   }
#endif

#if metodo==4

   float fhi;
   fhi = 2*PI*60*(1/(amostragem*60));

   Ka_A1[0] = cos(fhi);
   Ka_A1[1] = 1;
   Ka_A1[2] = cos(fhi);

   Ka_A2[0] = cos(3*fhi);
   Ka_A2[1] = 1;
   Ka_A2[2] = cos(3*fhi);

   Ka_B1[0] = cos(fhi);
   Ka_B1[1] = 1;
   Ka_B1[2] = cos(fhi);

   Ka_B2[0] = cos(3*fhi);
   Ka_B2[1] = cos(fhi);
   Ka_B2[2] = 1;
   Ka_B2[3] = cos(fhi);
   Ka_B2[4] = cos(3*fhi);

#endif
////////////////////////// SETTING BITS ///////////////////////////////
if(Amostragen_flag){
GpioDataRegs.GPBCLEAR.bit.GPIO62 = 1; // RESET
}else{
GpioDataRegs.GPBSET.bit.GPIO62 = 1; // RESET
}
GpioDataRegs.GPASET.bit.GPIO16 = 1; //PAR/SER?BYTE AD1
GpioDataRegs.GPASET.bit.GPIO17 = 1; //PAR/SER?BYTE AD2

GpioDataRegs.GPCCLEAR.bit.GPIO71 = 1; //DB14 AD1
GpioDataRegs.GPCCLEAR.bit.GPIO90 = 1; //DB15 AD1
GpioDataRegs.GPCCLEAR.bit.GPIO89 = 1; //DB14 AD2
GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1; //DB15 AD2

GpioDataRegs.GPCCLEAR.bit.GPIO92 = 1; //TESTE

#if RANGE == 10
    GpioDataRegs.GPASET.bit.GPIO4 = 1;  // RANGE
#else // 5 V
    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;  // RANGE
#endif
GpioDataRegs.GPCCLEAR.bit.GPIO72 = 1; //OSC0
GpioDataRegs.GPCCLEAR.bit.GPIO73 = 1; //OSC1
GpioDataRegs.GPCCLEAR.bit.GPIO78 = 1; //OSC2

GpioDataRegs.GPACLEAR.bit.GPIO10 = 1; //CONVER 1
GpioDataRegs.GPACLEAR.bit.GPIO11 = 1; //CONVER 2

GpioDataRegs.GPBSET.bit.GPIO42 = 1; //LED 1
GpioDataRegs.GPBSET.bit.GPIO43 = 1; //LED 2


lenght_pv = FFTLenWindow*Nciclos*amostragem;
//////////////////////////////////////////////////////////////////////////
// Abilita algumas interrupções
IER |= M_INT1;       // Enable CPU INT1
IER |= 0x1000;
IER |= 0x2000;
IER |= 0x100;                         // Enable CPU INT
IER |= 0x20;
// Enable global Interrupts and higher priority real-time debug events:
EINT;            // Habilita interrupcoes globais               - INTM
ERTM;            // Habilita interrupcoes globais de tempo real - DBGM

while(1){
    if(counter_flag>=counter_proc){// executa_FFT();
#if metodo==4
        counter_flag = 0;
        pv_hold = pv;

        int k;
        int jj;
        int i;

        for(k=0;k<FFTLenWindow*amostragem;k++){

        if((pv_hold+k-FFTLenWindow*amostragem)<0){
            jj = pv_hold+k-FFTLenWindow*amostragem+FFTLenWindow*amostragem*Nciclos;
        }else{
            jj = pv_hold+k-FFTLenWindow*amostragem;
        }

        for(i=0;i<ChannelsRead;i++){
            DATA_VECTOR_HOLD[i][k] = DATA_V_I_BASE[i]*DATA_VECTOR[i][jj];
        }
        }
        fft_complete(fase_REF,1);
#endif

#if(metodo == 0)
    	torres2014();  //ok
#endif
#if(metodo == 1)
    	erica2018();   //ok
#endif
#if(metodo == 2)
    	Aldair2020(); // ainda tem q testar e comparar com o do subramain
#endif
#if(metodo == 5)    //ok
    	subranain2012();
#endif
#if(metodo == 6)
    	executa_FFT();
#endif
#if(metodo == 3)
//    GpioDataRegs.GPCSET.bit.GPIO92 = 1; //TESTE
    brhama2012();
//    GpioDataRegs.GPCCLEAR.bit.GPIO92 = 1; //TESTE
#endif
#if(metodo == 4)
     kavi2018();
#endif
    counter_flag = 0;
    }
}
////****************************************************************************
//******************************************************************************
}
