/* le but du programme Go est de donner en sortie la valeur de la commande pour chaque moteur
on part de setTv pour le développer, en considérant que ce programme prend comme seul argument une commande de type Go 5) */

/**
Fonctionnement sommaire : le programme reçoit une commande type Go "d" en étant à la position (X,Y) avec l'angle theta
Il va donc avancer de d*cos(theta) sur X, et d*sin(theta) sur Y

2 premières hypothèses (fausses):
le robot est à l'arrêt quand il reçoit la commande, 
le robot s'arrête à l'instant où il ne reçoit plus de commande

Il faudrait introduire la notion d'erreur et de correction pour plus de précision 


**/


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
#define NB_ARGS             2               /* ->nombre d'arguments a passer en ligne de commande   : OK                         */
#define STR_LEN             64              /* ->taille des chaines par defaut                                               */
/*----------*/
/* globales */
/*----------*/

double *lpdb_Tv;            /* ->pointeur sur la commande comme sur SetTv         */
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
void usage( char *szPgmName)  /* je n'ai pas très bien compris l'usage de SzPgmName */
{
    if( szPgmName == NULL)
    {
        exit( -1 );
    };
    printf("%s <consigne> \n", szPgmName);

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
    
    /*................................................*/
    /* lien / creation aux zones de memoire partagees */
    /*................................................*/

    sprintf(szTargetAreaName,"%s%c", TARGET_BASENAME, cDriveID);
    /* zone de commande */
    if(( iFdTarget = shm_open(szTargetAreaName, O_RDWR, 0600)) < 0 )
    {
        fprintf(stderr,"%s.main() :  ERREUR ---> appel a shm_open() \n", argv[0]);
        fprintf(stderr,"             code1 = %d (%s)\n", errno, (char *)(strerror(errno)));
        exit( -errno );
    }
    else
    {
        printf("LIEN a la zone %s\n", szTargetAreaName);
    };
    if( ftruncate(iFdTarget, sizeof(double)) < 0 )
    {
        fprintf(stderr,"%s.main() :  ERREUR ---> appel a ftruncate() #1\n", argv[0]);
        fprintf(stderr,"             code2 = %d (%s)\n", errno, (char *)(strerror(errno)));
        exit( -errno );
    };
    if((lpdb_Tv = (double *)(mmap(NULL, sizeof(double), PROT_READ | PROT_WRITE, MAP_SHARED, iFdTarget, 0))) == MAP_FAILED )
    {
        fprintf(stderr,"%s.main() :  ERREUR ---> appel a mmap() #1\n", argv[0]);
        fprintf(stderr,"             code3 = %d (%s)\n", errno, (char *)(strerror(errno)));
        exit( -errno );
    };
    /*************************/
    /* fonctionnement normal */
    /*************************/
    *lpdb_Tv = Tv;
    return( 0 );   
}