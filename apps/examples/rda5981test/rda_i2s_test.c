#include <tinyara/config.h>
#include <stdio.h>
#include <unistd.h>

#include "../arch/arm/src/rda5981/rda5981x_i2s.h"

/*For I2S test*/ 
#define BUFFER_SIZE 240 
uint32_t rdata[BUFFER_SIZE] = {0}; 
uint32_t tdata[BUFFER_SIZE] = {0xF000F000}; 
i2s_t obj; 
i2s_cfg_t cfg; 
#if 0
static void *rx_thread(void *arg) 
{ 
   printf("begin rx\n"); 
   while (true) { 
        rda_i2s_int_recv(&obj, &rdata[0], BUFFER_SIZE); 
        if (I2S_ST_BUSY == obj.sw_rx.state) { 
            rda_i2s_sem_wait(i2s_rx_sem); 
        }
        printf("I2S a frame received,rdata[0]=%08X\n", rdata[0]); 
    } 
    return NULL; 
} 
#endif

static void *tx_thread(void *arg)
{
   printf("1tx\n");     
   rda_i2s_int_send(&obj, &tdata[0], BUFFER_SIZE);
   
   return NULL;
}

//pthread_t i2s_recv; 
pthread_t i2s_send; 

void spawn_tx_thread(void)
{
	pthread_attr_t attr1;
        struct sched_param sparam1;
        int status1;
        /* Initialize the attribute variable */
        status1 = pthread_attr_init(&attr1);
        if (status1 != 0) {
                printf("hello_tash : pthread_attr_init failed, status=%d\n", status1);
        }
        /* 1. set a priority */
        sparam1.sched_priority = 100;
        status1 = pthread_attr_setschedparam(&attr1, &sparam1);
        if (status1 != OK) {
                printf("hello_tash : pthread_attr_setschedparam failed, status=%d\n", status1);
	}                
       /* 2. set a stacksize */
        status1 = pthread_attr_setstacksize(&attr1, 4096);
        if (status1 != OK) {
                printf("hello_tash : pthread_attr_setstacksize failed, status=%d\n", status1);
        } 
	/* 3. create pthread with entry function */
        status1 = pthread_create(&i2s_send, &attr1, tx_thread, NULL);
        if (status1 != 0) {
              printf("hello_tash: pthread_create failed, status=%d\n", status1);
        }
}

#if 0
void spawn_rx_thread(void)
{
        pthread_attr_t attr;
        struct sched_param sparam;
        int status;
        /* Initialize the attribute variable */
        status = pthread_attr_init(&attr);
        if (status != 0) {
                printf("hello_tash : pthread_attr_init failed, status=%d\n", status);
        }
        /* 1. set a priority */
        sparam.sched_priority = 100;
        status = pthread_attr_setschedparam(&attr, &sparam);
                printf("hello_tash : pthread_attr_setschedparam failed, status=%d\n", status);
                
       /* 2. set a stacksize */
        status = pthread_attr_setstacksize(&attr, 4096);
        if (status != OK) {
                printf("hello_tash : pthread_attr_setstacksize failed, status=%d\n", status);
        }
        /* 3. create pthread with entry function */
        status = pthread_create(&i2s_recv, &attr, rx_thread, NULL);
        if (status != 0) {
                printf("hello_tash: pthread_create failed, status=%d\n", status);
        }
}
#endif

void i2s_test(void) /*master, in and out */ 
{ 
    printf("i2s_master_in test begin\n"); 
   
     
    uint32_t i; 
    for (i = 1; i < BUFFER_SIZE; i++) { 
        tdata[i] = tdata[i - 1] + 0x01000100; 
    }  
    cfg.mode              = I2S_MD_MASTER_RX; 
    cfg.rx.fs             = I2S_64FS; 
    cfg.rx.ws_polarity    = I2S_WS_NEG; 
    cfg.rx.std_mode       = I2S_STD_M; 
    cfg.rx.justified_mode = I2S_RIGHT_JM; 
    cfg.rx.data_len       = I2S_DL_16b; 
    cfg.rx.msb_lsb        = I2S_MSB; 
    
    cfg.tx.ws_polarity    = I2S_WS_NEG; 
    cfg.tx.std_mode       = I2S_STD_M; 
    cfg.tx.justified_mode = I2S_RIGHT_JM; 
    cfg.tx.data_len       = I2S_DL_16b; 
    cfg.tx.msb_lsb        = I2S_MSB; 
    cfg.tx.wrfifo_cntleft = I2S_WF_CNTLFT_8W; 
 
    rda_i2s_init(&obj); 
    rda_i2s_set_ws(&obj, 16000, 256); 
    rda_i2s_set_tx_channel(&obj, 2); 
    rda_i2s_set_rx_channel(&obj, 2); 
    rda_i2s_format(&obj, &cfg); 
 
    rda_i2s_enable_rx(&obj); 
    rda_i2s_enable_tx(&obj); 
 
/*create tx/rx thread*/ 
    spawn_tx_thread(); 

//    spawn_rx_thread(); 
    
// pthread_join(i2s_recv, NULL);
   pthread_join(i2s_send, NULL);

}
