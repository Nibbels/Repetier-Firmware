/* Isolated ATmega2560 simulation, no printer peripherals or real interrupts. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <simavr/sim_avr.h>
#include <simavr/sim_elf.h>

int main(int argc,char **argv) {
    if(argc!=5){fprintf(stderr,"usage: simavr-trace ELF phase fixture depth\n");return 2;}
    elf_firmware_t fw={0};
    if(elf_read_firmware(argv[1],&fw)<0)return 3;
    avr_t *avr=avr_make_mcu_by_name("atmega2560");
    if(!avr)return 4;
    avr_init(avr);
    avr->frequency=16000000;
    avr_load_firmware(avr,&fw);
    unsigned p=strtoul(argv[2],0,0), f=strtoul(argv[3],0,0), d=strtoul(argv[4],0,0);
    if(p>avr->ramend||f>avr->ramend||d>avr->ramend)return 5;
    uint8_t prior=0, prior_i=avr->sreg[7];
    while(avr->cycle<200000000ULL){
        int state=avr_run(avr);
        if(avr->sreg[7]!=prior_i){
            prior_i=avr->sreg[7];
            printf("I,%llu,%u\n",(unsigned long long)avr->cycle,prior_i);
        }
        uint8_t mark=avr->data[p];
        if(mark!=prior){
            prior=mark;
            printf("E,%llu,%u,%u,%u\n",(unsigned long long)avr->cycle,avr->data[f],avr->data[d],mark);
            if(mark==255){avr_terminate(avr);return 0;}
        }
        if(state==cpu_Crashed||state==cpu_Done){fprintf(stderr,"CPU state %d at %llu\n",state,(unsigned long long)avr->cycle);return 6;}
    }
    fprintf(stderr,"emulation cycle bound reached\n");return 7;
}
