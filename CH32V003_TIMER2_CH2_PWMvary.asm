# works fine on testing, PWM mode 1 ,R16_TIM2_CH2CVR decides pulse width , PD3 outputs the waveform , steps through 10% incremnts of PWM
#52.1 = 9600
#Default mapping (CH1/ETR/PD4, CH2/PD3,CH3/PC0, CH4/PD7).
fclk = 24000000   # 24Mhz RCO internal , AHB =8Mhz by default
state = 0x2000000C
#USARTx_TX Full-duplex mode Push-pull multiplexed outputs
#USARTx_RX Full-duplex mode Floating input or pull-up input
#PD5 -tx
#PD6 -rx
include CH32V003_reg1.asm

vtable:
	j reset_handler		#  longs 0x00000000 # RESERVED 0
align 4
  longs   0x00000000 # RESERVED 1
  longs   0x00000000 #pack <l longs NMI_IRQhandler
  longs   0x00000000 #pack <l HardFault_IRQhandler
  longs   0x00000000 # RESERVED 4
  longs   0x00000000 # RESERVED 5
  longs   0x00000000 # RESERVED 6
  longs   0x00000000 # RESERVED 7
  longs   0x00000000 # RESERVED 8
  longs   0x00000000 # RESERVED 9
  longs   0x00000000 # RESERVED 10
  longs   0x00000000 # RESERVED 11
  longs   0x00000000 # pack <l SysTick_IRQhandler	#; place the address of the mtime ISR subroutine in the vector table position 7,assembler will store isr address here, longs 0x00000000 # RESERVED 12	
  longs   0x00000000 # RESERVED 13
  longs   0x00000000 #pack <l SW_Software_IRQhandler
  longs   0x00000000 # RESERVED 15
  longs   0x00000000 #pack <l WWDG_IRQhandler
  longs   0x00000000 #pack <l PVD_IRQhandler
  longs   0x00000000 #pack <l FLASH_IRQhandler
  longs   0x00000000 #pack <l RCC_IRQhandler
  longs   0x00000000 #pack <l EXTI7_0_IRQhandler
  longs   0x00000000 #pack <l AWU_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH1_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH2_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH3_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH4_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH5_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH6_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH7_IRQhandler
  longs   0x00000000 #pack <l ADC1_IRQhandler
  longs   0x00000000 #pack <l I2C1_EV_IRQhandler
  longs   0x00000000 #pack <l I2C1_ER_IRQhandler
  longs   0x00000000 #pack <l USART1_IRQhandler
  longs   0x00000000 #pack <l SPI1_IRQhandler
  longs   0x00000000 #pack <l TIM1BRK_IRQhandler
  longs   0x00000000 #pack <l TIM1UP_IRQhandler
  longs   0x00000000 #pack <l TIM1TRG_COM_IRQhandler
  longs   0x00000000 #pack <l TIM1CC_IRQhandler
pack <l TIM2_IRQhandler

reset_handler:


    	li sp, STACK			# load stack pointer with stack end address
	 
    	li t0, vtable			#BASEADDR[31:2],The interrupt vector table base address,which needs to be 1KB aligned
    	ori t0, t0, 3			#BASEADDR[31:2],1: Identify by absolute address,1: Address offset based on interrupt number *4
    	#csrrw zero,t0, mtvec		# write to mtvec
	longs 0x30529073  
    
   	li t0,main
	longs 0x34129073          	#csrw	mepc,t0 :mepc updated with address of main
	longs 0x30200073		# mret ( return from interrupt)	.
  
	align 4
main:
	nop


#enable periphrel clocks
	li x10,R32_RCC_APB2PCENR	# load address of APB2PCENR register to x10 ,for enabling GPIO A,D,C peripherals
	lw x11,0(x10)			# load contents from peripheral register R32_RCC_APB2PCENR pointed by x10
	li x7,((1<<2)|(1<<4)|(1<<5)|(1<<14)|(1<<0))	# 1<<IOPA_EN,1<<IOPC_EN,1<<IOPD_EN,1<<USART_EN
	or x11,x11,x7			# or values 
	sw x11,0(10)			# store modified enable values in R32_RCC_APB2PCENR
	li x10,R32_RCC_APB1PCENR
	lw x11,0(x10)
	ori,x11,x11,(1<<0)		# timer2 clock enable
	sw x11,0(x10)

#configure GPIO 
	li x10,R32_GPIOD_CFGLR		# load pointer x10 with address of R32_GPIOD_CFGLR , GPIO configuration register
	lw x11,0(x10)			# load contents from register pointed by x10
	li x7,~(0xf<<12)		# clear pd3, we need to setup  pd3 for pwm
	and x11,x11,x7			# clear pd3 mode and cnf bits for selected pin D3
	li x7,(0xB<<12)			# pd3 = pd3 multiplex pushpull for ch2 pwm capture compare output
	or x11,x11,x7			# OR value to register
	sw x11,0(x10)			# store in R32_GPIOD_CFGLR


timer_for_CCoutput:
	li x10,R16_TIM2_PSC
	li x11,(7999)			# fck_PSC/PSC[15:0]+1,8MHz/7+1 =1MHz
	sw x11,0(x10)
	li x10,R16_TIM2_ATRLR
	li x11,(999)			# max reload
	sw x11,0(x10)
	li x10,R16_TIM2_CH2CVR
	li x11,99			# 10% duty cycle
	sw x11,0(x10)
	li x10,R16_TIM2_DMAINTENR
	lw x11,0(x10)
	li x7,(1<<2)			# enable CC2IE interrupt,ch2 compare
	or x11,x11,x7
	sw x11,0(x10)
	li x10,R16_TIM2_CHCTLR1
	lw x11,0(x10)
	li x7,((6<<12)|(1<<11))		# CH2 mode-1(0b110), CH2 output compare preload enable OC2PE,
	or x11,x11,x7
	sw x11,0(x10)
	li x10,R16_TIM2_CCER
	lw x11,0(x10)
	li x7,(1<<4)			# enable cc2 as output
	or x11,x11,x7
	sw x11,0(x10)
	li x10,R16_TIM2_CTLR1
	lw x11,0(x10)
	li x7,((0<<1)|(1<<2)|(1<<7))	# UDIS =0 ,URS =1,ARPE=1,
	or x11,x11,x7
	sw x11,0(x10)
	li x10,R16_TIM2_SWEVGR
	lw x11,0(x10)
	li x7,(1<<0)			# 1<<UG
	or x11,x11,x7
	sw x11,0(x10)
	li x10,R16_TIM2_INTFR
	lw x11,0(x10)
	li x7,0xFFFFFFFA		# 0<<UIF ,0<<CC2IF clear update interrupt flag
	and x11,x11,x7
	sw x11,0(x10)
	li x10,R16_TIM2_CTLR1
	lw x11,0(x10)
	ori x11,x11,(1<<0)		# enable timer 2
	sw x11,0(x10)


PFIC_CONFIG:
	li x10,R32_PFIC_CFGR		# reset core PFIC register for interrupts
	lw x11,0(x10)
	li x7,((PFIC_KEY3<<16)|(1<<7))	# key3  and SYSRESET , reference manual tells to do it
	or x11,x11,x7
	sw x11,0(x10)			# store back new values

	li x10,R32_PFIC_IENR2		# PFIC Interrupt Enable in core PFIC
	lw x11,0(x10)
	li x7,(1<<6 )			# enabled  TIM2 interrupts
	or x11,x11,x7
	sw x11,0(x10)			# store back new values

# enabling GLOBAL INTERRUPTS
	li t0, 0x88			# load MPIE and MIE bits , 1<<MIE in mstatus is enabling GLOBAL INTERRUPTS
	longs 0x30029073        	#csrw	mstatus,t0 ,manually assembled opcade to do csrrw the values in t0,  

	li x10,state			# state register stores the current capture compare value aka on time %
	li x11,99			# load 10% to start with (999/99= 10%)
	sw x11,0(x10)			# store in state register(sram)
finish:
	call delay			# delay of approximate 10 seconds at 24Mhz HSI & Pclk 8Mhz
	li x10,state			# x10 point to state register
	li x8,999			# load x8 with 999 which is 100% duty cycle value, when state reached 100% we need to start over from 10%
	lw x11,0(x10)			# copy current duty cycle in state register to x11
	bne x8,x11,ook			# if x11 is not equal to 100% duty cycle branch to ook label to increment the duty cycle by 10%
	li x11,-1			# if state register has 100% duty cycle load x11 with -1. next step will add 100 as part of incremet cycle. -1 esures the seed value of 99
ook:
	li x7,100			# load 100 in x7, every 100 addition increases the duty cycle value by 10%
	add x11,x11,x7			# add the current value of state in x11 with 100 in x7 to increse the duty cycle by 10%
	sw x11,0(x10)			# store back x11 to state register
	li x10,R16_TIM2_CH2CVR		# x10 points to R16_TIM2_CH2CVR, capture compare value register, 
	sw x11,0(x10)			# store modified value of new PWM duty cycle in R16_TIM2_CH2CVR
	j finish			# loop to finish
###################################################
delay:	
	addi sp,sp,-8			# move sp 2 words
	sw ra,0(sp)			# push ra
	sw x6,4(sp)			# push x6
	li x6,20000000			# load an arbitarary value 20000000 to t1 register		
dloop:
	addi x6,x6,-1			# subtract 1 from t1
	bne x6,zero,dloop		# if t1 not equal to 0 branch to label loop
	lw x6,4(sp)			# pop x6
	lw ra,0(sp)			# pop ra
	addi sp,sp,8			# sp back 2 words
	ret				# return to caller
##########################################################################################################
##########################################################
##########################################################

TIM2_IRQhandler:
	#clear exti interrupt flag
	addi sp,sp,-60    		# push all registers
	sw x15,56(sp)
	sw x14,52(sp)
	sw x13,48(sp)
	sw x12,44(sp)
	sw x11,40(sp)
	sw x10,36(sp)
	sw x9,32(sp)
	sw x8,28(sp)
	sw x7,24(sp)
	sw x6,20(sp)
	sw x5,16(sp)
	sw x4,12(sp)
	sw x3,8(sp)
	sw x2,4(sp)
	sw x1,0(sp)
	
###########
	li x10,R16_TIM2_INTFR
	lw x11,0(x10)
	li x7,0xFFFFFFFA			# 0<<UIF| 0<<CC2IF
	and x11,x11,x7
	sw x11,0(x10)
############		
	lw x1,0(sp)
	lw x2,4(sp)
	lw x3,8(sp)
	lw x4,12(sp)
	lw x5,16(sp)
	lw x6,20(sp)
	lw x7,24(sp)
	lw x8,28(sp)
	lw x9,32(sp)
	lw x10,36(sp)
	lw x11,40(sp)
	lw x12,44(sp)
	lw x13,48(sp)
	lw x14,52(sp)
	lw x15,56(sp)
	addi sp,sp,60
	longs 0x30200073		# mret (manually assembled opcode for mret as per RISCV spec)
	
#################