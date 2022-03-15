/*===================================*/
/* utilitaire pour imposer la valeur */
/* de la consigne a un moteur        */
/* ----------------------------------*/
/* J.BOONAERT AMSE 2021-2022         */
/*===================================*/
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/time.h>
/*............*/
/* constantes */
/*............*/
#define TARGET_BASENAME    "TARGET_"
#define NB_ARGS             3               /* ->nombre d'arguments a passer en ligne de commande                            */
#define STR_LEN             64              /* ->taille des chaines par defaut                                               */
/*----------*/
/* globales */
/*----------*/
double *lpdb_Tv;            /* ->pointeur sur la commande          */
/*--------------*/
/* declarations */
/*--------------*/
void usage( char *);        /* ->aide de ce programme              */
/*-------------*/
/* definitions */
/*-------------*/
/*&&&&&&&&&&&&&&&&&&&&&&*/
/* aide de ce programme */
/*&&&&&&&&&&&&&&&&&&&&&&*/
void usage( char *szPgmName)
{
    if( szPgmName == NULL)
    {
        exit( -1 );
    };
    printf("%s <consigne> <drive>\n", szPgmName);
    printf("   avec <drive> = L | R \n");
}
/*######*/
/* MAIN */
/*######*/
int main( int argc, char *argv[])
{
    char    cDriveID;                       /* ->caractere pour identifier le moteur            */
    char    szTargetAreaName[STR_LEN];      /* ->nom de la zone contenant la commande           */
    int     iFdTarget;                      /* ->descripteur pour la zone de commande           */
    double  *lpdb_Tv;                       /* ->pointeur sur la zone partagee                  */
    double  Tv;                             /* ->valeur a appliquer a la consigne               */
    /*.......*/
    /* check */
    /*.......*/
    if( argc != NB_ARGS)
    {
        usage(argv[0]);
        return( 0 );
    };
    /*............................*/
    /* recuperation des arguments */
    /*............................*/
    if( sscanf(argv[1],"%lf",&Tv) == 0 )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> l'argument #1 doit etre reel\n", argv[0]);
        usage(argv[0]);
        return( 0 );
    };
    if( sscanf(argv[2],"%c",&cDriveID) == 0 )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> l'argument #2 doit etre un caractere\n", argv[0]);
        usage(argv[0]);
        return( 0 );
    };
    /*................................................*/
    /* lien / creation aux zones de memoire partagees */
    /*................................................*/
    sprintf(szTargetAreaName,"%s%c", TARGET_BASENAME, cDriveID);
    /* zone de commande */
    if(( iFdTarget = shm_open(szTargetAreaName, O_RDWR, 0600)) < 0 )
    {
        fprintf(stderr,"%s.main() :  ERREUR ---> appel a shm_open() \n", argv[0]);
        fprintf(stderr,"             code = %d (%s)\n", errno, (char *)(strerror(errno)));
        exit( -errno );
    }
    else
    {
        printf("LIEN a la zone %s\n", szTargetAreaName);
    };
    if( ftruncate(iFdTarget, sizeof(double)) < 0 )
    {
        fprintf(stderr,"%s.main() :  ERREUR ---> appel a ftruncate() #1\n", argv[0]);
        fprintf(stderr,"             code = %d (%s)\n", errno, (char *)(strerror(errno)));
        exit( -errno );
    };
    if((lpdb_Tv = (double *)(mmap(NULL, sizeof(double), PROT_READ | PROT_WRITE, MAP_SHARED, iFdTarget, 0))) == MAP_FAILED )
    {
        fprintf(stderr,"%s.main() :  ERREUR ---> appel a mmap() #1\n", argv[0]);
        fprintf(stderr,"             code = %d (%s)\n", errno, (char *)(strerror(errno)));
        exit( -errno );
    };
    /*************************/
    /* fonctionnement normal */
    /*************************/
    *lpdb_Tv = Tv;
    return( 0 );   
}