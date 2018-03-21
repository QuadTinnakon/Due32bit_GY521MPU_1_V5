/*
project_Quad 32 bit Arduino Due  Timer_due32.h
1. stabilized quadrotor 
by: tinnakon kheowree 
0860540582
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com/
https://www.facebook.com/tinnakonza

ISR/IRQ	TC     Channel	Due pins
T0	TC0	0	2, 13
T1	TC0	1	60, 61
T2	TC0	2	58
T3	TC1	0	none  <- this line in the example above
T4	TC1	1	none
T5	TC1	2	none
T6	TC2	0	4, 5
T7	TC2	1	3, 10
T8	TC2	2	11, 12
*/
void TC3_Handler()//Loop 1000 Hz
{
       TC_GetStatus(TC1, 0);
}
void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
       pmc_set_writeprotect(false);
       pmc_enable_periph_clk((uint32_t)irq);
       TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
       uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
       TC_SetRA(tc, channel, rc/2); //50% high, 50% low
       TC_SetRC(tc, channel, rc);
       TC_Start(tc, channel);
       tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
       tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
       NVIC_EnableIRQ(irq);
}
