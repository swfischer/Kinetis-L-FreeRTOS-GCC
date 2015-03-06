#define FLAG_SET(BitNumber, Register)        (Register |=(1<<BitNumber))
#define FLAG_CLR(BitNumber, Register)        (Register &=~(1<<BitNumber))
#define FLAG_CHK(BitNumber, Register)        (Register & (1<<BitNumber))

#define _USB     0
