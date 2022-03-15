/*===================================*/
/* utilitaire pour reinitialiser     */
/* l'etat d'un moteur a 0            */
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
#define STATE_BASENAME    "STATE_"
#define NB_ARGS         2               /* ->nombre d'arguments a passer en ligne de commande                            */
#define STR_LEN         64              /* ->taille des chaines par defaut                                               */
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
    printf("%s <drive>\n", szPgmName);
    printf("   avec <drive> = L | R \n");
}
/*######*/
/* MAIN */
/*######*/
int main( int argc, char *argv[])
{
    char    cDriveID;                       /* ->caractere pour identifier le moteur            */
    char    szStateAreaName[STR_LEN];       /* ->nom de la zone contenant la commande           */
    int     iFdState;                       /* ->descripteur pour la zone de commande           */
    double  *lpdb_state;                    /* ->pointeur sur la zone partagee                  */
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
    if( sscanf(argv[1],"%c",&cDriveID) == 0 )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> l'argument #1 doit etre un caractere\n", argv[0]);
        usage(argv[0]);
        return( 0 );
    };
    /*................................................*/
    /* lien / creation aux zones de memoire partagees */
    /*................................................*/
    sprintf(szStateAreaName,"%s%c", STATE_BASENAME, cDriveID);
    /* zone de commande */
    if(( iFdState = shm_open(szStateAreaName, O_RDWR, 0600)) < 0 )
    {
        fprintf(stderr,"%s.main() :  ERREUR ---> appel a shm_open() \n", argv[0]);
        fprintf(stderr,"             code = %d (%s)\n", errno, (char *)(strerror(errno)));
        exit( -errno );
    }
    else
    {
        printf("LIEN a la zone %s\n", szStateAreaName);
    };
    if( ftruncate(iFdState, 2 * sizeof(double)) < 0 )
    {
        fprintf(stderr,"%s.main() :  ERREUR ---> appel a ftruncate() #1\n", argv[0]);
        fprintf(stderr,"             code = %d (%s)\n", errno, (char *)(strerror(errno)));
        exit( -errno );
    };
    if((lpdb_state = (double *)(mmap(NULL, 2 * sizeof(double), PROT_READ | PROT_WRITE, MAP_SHARED, iFdState, 0))) == MAP_FAILED )
    {
        fprintf(stderr,"%s.main() :  ERREUR ---> appel a mmap() #1\n", argv[0]);
        fprintf(stderr,"             code = %d (%s)\n", errno, (char *)(strerror(errno)));
        exit( -errno );
    };
    /*************************/
    /* fonctionnement normal */
    /*************************/
    memset(lpdb_state,0, 2 * sizeof(double));
    return( 0 );   
}