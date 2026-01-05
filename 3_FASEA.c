#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#define QUANTUM 2
#define HELBIDE_BUSA 24
#define TLB_TAMAINA 14
#define MEM_FISICOA_SIZE (1 << 24)
#define PAGE_SIZE 4096
#define NUM_PAGES (MEM_FISICOA_SIZE / PAGE_SIZE)
#define KERNEL_MEM_SIZE (1 << 20) 
#define PT_ENTRIES 64
pid_t pid;
pthread_mutex_t mutex;
pthread_cond_t cond_TICK;
uint8_t memoria_fisikoa[MEM_FISICOA_SIZE];

uint32_t next_kernel_mem_libre=0;
uint32_t next_user_mem_libre= KERNEL_MEM_SIZE;
int done=0;
const int Prozesu_kop_Max=25;
const int maiztasuna=100000;
int tenp_kop=2;
int Prozesu_kop= 0;
bool Proccess_bukatu=false;
int tick=0;
int scheduling_mode = 1;
// 0 = round robin
// 1 = lehentasun estatikoa
// 2 = lehentasun dinamikoa
typedef struct {
    uint32_t pgb;
    uint32_t code;
    uint32_t data; 
} MemoryManagement;

typedef struct {
    pid_t pid;
    int remaining_time;
    int lehentasuna;
    MemoryManagement mm;
    uint16_t IR;
    uint16_t PC;
} PCB;
typedef struct {
    uint32_t orri_logikoa; 
    uint32_t marko_fisikoa;
    bool valid;
} TLB_mota;

typedef struct {
    TLB_mota TLB[TLB_TAMAINA];
    uint32_t PTBR;
    uint32_t IR;
    uint32_t PC;
} Hardware;
Hardware cpu;

typedef struct {
    uint32_t marco_fisikoa;
    bool valid;
} Orri_taulaM;
int wait_count=0;
PCB wait[25];
int ready_count=0;
PCB ready[25];
PCB running;
int erabilitako_quantum=0;

Orri_taulaM* sortu_orri_taula() {
    Orri_taulaM* orri_taula = (Orri_taulaM*) &memoria_fisikoa[next_kernel_mem_libre];
    

    for (int i=0; i<PT_ENTRIES; i++) { 
        orri_taula[i].valid=false;
        orri_taula[i].marco_fisikoa=0;
    }
    next_kernel_mem_libre+= sizeof(Orri_taulaM) * PT_ENTRIES;

    return orri_taula;
} 


void* erlojua(void* arg);
void* temporizadorea(void* arg);
void* proccesGenerator(void* arg);
void* Scheduler(void* arg);
void kargatu_kodea_eta_datuak(PCB *p, const char *fitxategia);



pid_t Procces_queue[25];


int main()
{
    cpu.PTBR =0; 
    cpu.PC=0;
    cpu.IR=0;
    
    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&cond_TICK, NULL);

    pthread_t hari_erlojua;
    pthread_t hari_temporizadorea[tenp_kop];
    pthread_t hari_proccess_generator;
    pthread_t hari_scheduler;

    pthread_create(&hari_erlojua, NULL, erlojua, NULL);
    
    for (int i=0; i<tenp_kop; i++) {
        pthread_create(&hari_temporizadorea[i], NULL, temporizadorea, NULL); 
    }
    pthread_create(&hari_proccess_generator, NULL, proccesGenerator, NULL);

    pthread_create(&hari_scheduler, NULL, Scheduler, NULL);



    pthread_join(hari_erlojua, NULL);
    for (int i=0; i< tenp_kop; i++) {
        pthread_join(hari_temporizadorea[i], NULL);
    }
    pthread_join(hari_proccess_generator, NULL);
    pthread_join(hari_scheduler, NULL);


    for (int i=0; i < TLB_TAMAINA; i++ ) {
        cpu.TLB[i].valid = false;
    }
}
void ready_ilaran_sartu(PCB p) {
    if (ready_count < 25) {
    ready[ready_count]=p;
    ready_count++;
    } else {
        printf("Ilara bete egin da");
    } 
}
PCB ready_ilaratik_atera() {
    PCB gorde;
    gorde= ready[0];
    for (int i=0; i<ready_count - 1; i++) {
        ready[i]= ready[i+1];
    }
    ready_count--;
    return gorde;
}
void wait_ilaran_sartu(PCB pid) {
    if (wait_count < 25) {
    wait[wait_count]=pid;
    wait_count++; 
    } else {
        printf("Wait ilara bete egin da");
    }
}
PCB wait_ilaratik_atera() {
    PCB gorde;
    gorde= wait[0];
    for (int i=0; i<wait_count - 1; i++) {
        wait[i]= wait[i+1];
    }
    wait_count--;
    return gorde;
}

void lehentasunaren_arabera_ordenatu () {
    if (ready_count > 1) {
        for (int i=0; i< ready_count; i++) {
            for (int j=i+1; j< ready_count; j++) {
                if (ready[i].lehentasuna < ready[j].lehentasuna) {
                    PCB aux= ready[i]; 
                    ready[i]= ready[j];
                    ready[j]= aux;
                }
            }
        }
    }
}

void lehentasuna_jeitsi (PCB *p) { 
    if (p->lehentasuna > 0) {
        p->lehentasuna--;
    }
}

void *erlojua(void* arg)
{
    while (1)
    {
        usleep(maiztasuna * 5);
        pthread_mutex_lock(&mutex);
        tick++;
        printf(">>>Erlojua: tick %d\n", tick);
        pthread_cond_broadcast(&cond_TICK); 
        pthread_mutex_unlock(&mutex);

    }
    
}
void *temporizadorea(void* arg)
{
    while(1) {
        pthread_mutex_lock(&mutex);
        int current_tick = tick;

        while (tick == current_tick)
            pthread_cond_wait(&cond_TICK, &mutex);
        pthread_mutex_unlock(&mutex);

        printf("TeNporizadorea aktibatuta, done=%d\n", tick);
        usleep(maiztasuna);

    }
}
void *proccesGenerator(void *arg)
{
    int current_tick=0;
    while (1) {
        pthread_mutex_lock(&mutex); 
        while (tick == current_tick)
            pthread_cond_wait(&cond_TICK, &mutex);

        current_tick=tick;
        
        if (Prozesu_kop_Max > Prozesu_kop) {
            Prozesu_kop++;
            PCB berria;
            berria.pid=Prozesu_kop;
            berria.remaining_time=2+ rand() %15;
            berria.lehentasuna = rand() % 5;

            Orri_taulaM *pt = sortu_orri_taula();
            berria.mm.pgb = (uint32_t)((uint8_t *)pt - memoria_fisikoa);

            kargatu_kodea_eta_datuak(&berria, "proba");

            ready_ilaran_sartu(berria);
            printf("Prozesua ondo sortu da: Bere (PID) honakoa da: PID=%d Remainiting Time=%d, Lehentasuna=%d (Prozesu Kopurua=%d)\n", berria.pid, berria.remaining_time, berria.lehentasuna, Prozesu_kop);
        }
    pthread_mutex_unlock(&mutex);
    
    if (Prozesu_kop >= Prozesu_kop_Max)
        break;
    usleep(maiztasuna);
    }
    return NULL;
}


void *Scheduler(void *arg)
{
    running.pid = -1;
    running.remaining_time =0;
    running.lehentasuna=0;
    
    while(1) {
        pthread_mutex_lock(&mutex);

        int current_tick = tick;
        while (tick == current_tick)
            pthread_cond_wait(&cond_TICK, &mutex);


        if (scheduling_mode == 0) { //Round Robin
            if (running.pid == -1 && ready_count > 0) {
                running = ready_ilaratik_atera();
                erabilitako_quantum = 0;
                printf(">>>Scheduler: PID %d CPU-ra sartu da\n", running.pid);
            }

            if (running.pid != -1) {
                running.remaining_time--;
                erabilitako_quantum++;

                printf(">>>RUNNING EGOERA: PID %d GELDITZEN ZAION DENBORA=%d ERABILI DUEN QUANTUMA=%d\n",
                    running.pid, running.remaining_time, erabilitako_quantum);

                if (running.remaining_time <= 0) {
                printf(">>>Scheduler: PID %d amaitu da\n", running.pid);
                running.pid = -1;
                erabilitako_quantum = 0;
                }
                else if (erabilitako_quantum >= QUANTUM) {
                    printf(">>>Scheduler: PID %d quantum agortuta\n", running.pid);
                    ready_ilaran_sartu(running);
                    running.pid = -1;
                    erabilitako_quantum = 0;
                }
            }
        }
        else if (scheduling_mode == 1) { //lehentasun estaticoa
            lehentasunaren_arabera_ordenatu();
            
            if (running.pid == -1 && ready_count > 0) {
                running= ready_ilaratik_atera();
                printf("Scheduler: PID %d CPU-ra sartu da, lehentasuna=%d \n", running.pid, running.lehentasuna);
            }
            if (running.pid != -1 && ready_count > 0) {
                if (running.lehentasuna < ready[0].lehentasuna) {
                    printf("Scheduler: PID %d kanpora atera da eta PID %d sartu da (lehentasun handiagoa). \n", running.pid, ready[0].pid);

                    ready_ilaran_sartu(running);
                    running = ready_ilaratik_atera();
                }
            }
            if (running.pid != -1) {
                running.remaining_time--;
                printf("RUNNING: PID %d Remaining time=%d Lehentasuna=%d\n", running.pid, running.remaining_time, running.lehentasuna);

                if (running.remaining_time <= 0) {
                    printf("PID %d amaitu da\n", running.pid);
                    running.pid = -1;
                }
            }
        }
        else if (scheduling_mode == 2) { //lehentasun dinamikoa
            lehentasunaren_arabera_ordenatu();
            if (running.pid == -1 && ready_count > 0) {
                running= ready_ilaratik_atera();
                erabilitako_quantum=0;
                printf("Scheduler: PID %d CPU-ra sartu da, Lehentasuna=%d \n", running.pid, running.lehentasuna);
            }
            if (running.pid != -1 && ready_count > 0) {
                if (running.lehentasuna < ready[0].lehentasuna) {
                    printf("Scheduler: PID %d kanpora atera da eta PID %d sartu da (lehentasun handiagoa). \n", running.pid, ready[0].pid);

                    ready_ilaran_sartu(running);
                    running = ready_ilaratik_atera();
                }

            }
            if (running.pid != -1) {
                running.remaining_time--;
                lehentasuna_jeitsi(&running);
                printf("RUNNING: PID %d Remaining time=%d Lehentasuna=%d\n", running.pid, running.remaining_time, running.lehentasuna);

                

                if (running.remaining_time <= 0) {
                    printf("Scheduler: PID %d amaitu da \n", running.pid);
                    running.pid=-1;
                }
            }

        }
        
        pthread_mutex_unlock(&mutex);
    }
    return NULL;
}
void kargatu_kodea_eta_datuak(PCB *p, const char *fitxategia) {
    FILE *f = fopen(fitxategia, "rb");
    if (f == NULL) {
        printf("Ezin izan da fitxategia ireki: %s\n", fitxategia);
        return;
    }
    p->mm.code = 0x00000000;
    p->mm.data = 0x00001000;

    uint32_t code_helbide_fisikoa = next_user_mem_libre;
    fread(&memoria_fisikoa[code_helbide_fisikoa], 1, PAGE_SIZE, f);
    next_user_mem_libre += PAGE_SIZE;

    uint32_t data_helbide_fisikoa = next_user_mem_libre;
    fread(&memoria_fisikoa[data_helbide_fisikoa], 1, PAGE_SIZE, f);
    next_user_mem_libre += PAGE_SIZE;

    fclose(f);
    Orri_taulaM *pt = (Orri_taulaM *) &memoria_fisikoa[p->mm.pgb];
    
    pt[0].marco_fisikoa = code_helbide_fisikoa / PAGE_SIZE;
    pt[0].valid = true;

    pt[1].marco_fisikoa = data_helbide_fisikoa/ PAGE_SIZE;
    pt[1].valid = true;

}




