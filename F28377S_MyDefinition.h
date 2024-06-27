

void configureDAC(Uint16 dac_num);
void PlotDAC(void);
void spi_fifo_init(void);
void timer_config(Uint16 timerzero_t_us, Uint16 timerum_t_us, Uint16 timerdois_t_us);
void Gpioconfig(void);
void external_interrupt(void);
void Conversation_Done(void);
void Save_data(void);
void mcbsp_a_init_dlb(void);
void mcbsp_b_init_dlb(void);
void scia_init(void);


__interrupt void cpu_timer0_isr(void);  // timer auxiliar contador
__interrupt void cpu_timer1_isr(void);  // Timer de amostragem
__interrupt void cpu_timer2_isr(void);  // Relogio Global


//__interrupt void Mcbsp_RxINTA_ISR(void);
//__interrupt void Mcbsp_RxINTB_ISR(void);
//
//__interrupt void xint1_isr(void);
//__interrupt void xint2_isr(void);
//
//__interrupt void sciaTxFifoIsr(void);
//__interrupt void sciaRxFifoIsr(void);


