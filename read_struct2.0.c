#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define NUM_PACKETS 50


typedef struct
{
    int16_t acclsmx;
    int16_t acclsmy;
    int16_t acclsmz;
    int16_t anglsmx;
    int16_t anglsmy;
    int16_t anglsmz;
    uint16_t A0;
    uint16_t A1;
    uint16_t A2;
    uint16_t pulses_chan1;
    uint16_t pulses_chan2;
    uint32_t timestamp;
} packet;

int main()
{
    char error, first = 0;
    int  RUN, part = 0;
    char filename[50];
    char foldername[30];
    FILE *f, *fp;

    printf("Insira o nome da pasta em que se encontram os dados: ");
    scanf(" %s", foldername);
    while(1)
    {
        error = 0;
        part = 0;
        printf("Insira o número da corrida a ser lida (negativo para sair): ");
        scanf(" %d", &RUN);

        if(RUN < 0)
            break;

        sprintf(filename, "%s/RUN%d.csv", foldername, RUN);
        f = fopen(filename, "wt");
        // printf("file = %ld\r\n", f);
        
        fprintf(f, "lsmaccx,lsmaccy,lsmaccz,lsmangx,lsmangy,lsmangz,a0,a1,a2,f1,f2,timestamp\n");
    
            char name[70];
            sprintf(name, "%s/%s%d/%s%d", foldername,"RUN", RUN, "part", part+1);
            printf("filename = %s\n", name);
            fp = fopen(name, "r");
            packet x[NUM_PACKETS];
            
            if (fp == NULL)
            {
                error = 1;
                break;
            }   
        
            printf("~~~~~~~~PART %d ~~~~~~~~", part);
            
            while(1){
  	
	    fread((void *)x, sizeof(packet), 1, fp); 
	    fprintf(f, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", x->acclsmx, x->acclsmy, x->acclsmz, x->anglsmx, x->anglsmy, x->anglsmz, x->A0, x->A1, x->A2, 
	    x->pulses_chan1, x->pulses_chan2, x->timestamp);
            if (feof(fp)) break;
	    }
            fclose(fp);
            fp = NULL;
            fclose(f);
            f = NULL;
    }
    
    return 0;
}
