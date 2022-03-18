/*=======================================*/
/* Application du modele cinematique     */
/* du robot pour determiner ses vitesses */
/* lineaire et angulaires                */
/* --------------------------------------*/
/* J.BOONAERT AMSE 2021-2022             */
/*=======================================*/
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
#define OFFSET_W        0               /* ->indice pour acceder a la vitesse angulaire d'un rotor                       */
#define STATE_L             "STATE_L"   /* ->etat du moteur gauche                                                       */
#define STATE_R             "STATE_R"   /* ->etat du moteur droit                                                        */
#define VELOCITY            "VELOCITY"  /* ->nom de la zone partagee pour les vitesses lineaire et angulaire             */
#define NB_ARGS         4               /* ->nombre d'arguments a passer en ligne de commande                            */
#define REFRESH_RATE    5               /* ->nombre d'iterations a realiser pour 1 affichage de l'etat et de la commande */
#define OFFSET_VC       0               /* ->offset sur la zone d'etat pour acceder a la vitesse angulaire               */
#define OFFSET_WC       1               /* ->offset sur la zone d'etat pour acceder a l'intensite                        */
#define STR_LEN         64              /* ->taille des chaines par defaut                                               */
/*----------*/
/* globales */
/*----------*/
int     GoOn = 1;           /* ->controle d'execution du processus             */
int     iCount = 0;         /* ->comptage du nombre d'iterations               */
double  *lpdb_wL;           /* ->pointeur sur la vitesse angulaire roue gauche */    
double  *lpdb_wR;           /* ->pointeur sur la vitesse angulaire roue droite */
double  *lpdb_vel;          /* ->pointeur sur la zone partagee des vitesses    */
double  Te;                 /* ->periode d'echantillonnage                     */
double  R0;                 /* ->rayon commun des roues du robot               */
double  w;                  /* ->entraxe des roues                             */
/*--------------*/
/* declarations */
/*--------------*/
void usage( char *);        /* ->aide de ce programme                            */
void *Link2SharedMem( char *, int, int *, int);
                            /* ->creation ou lien a une zone de memoire          */
                            /*   partagee                                        */
void updateVelocity(void);  /* ->mise a jour des vitesses lineaires et angulaire */
void SignalHandler(int);    /* ->gestionnaire de signale                         */
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
    printf("%s <R0> <W> <Periode d'ech.> <drive>\n", szPgmName);
    printf("   avec <drive> = L | R \n");
}
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/* creation ou lien a une zone de memoire */
/* partagee.                              */
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
void *Link2SharedMem(   char *szAreaName,           /* ->nom de la zone partagee        */
                        int iSize,                  /* ->taille de la zone (en octets)  */
                        int *iFd,                   /* ->descripteur associe a la zone  */
                        int iCreate            )    /* ->cree la zone (1) si necessaire */
{
    void *vAddr;             /* ->pointeur sur la zone partagee creee / liee */
    /*.......*/
    /* check */
    /*.......*/
    if( szAreaName == NULL )
    {
        fprintf(stderr,"Link2SharedMem() : ERREUR ---> pointeur NULL passe en argument #1\n");
        return( NULL );
    };
    if( iFd == NULL )
    {
        fprintf(stderr,"Link2SharedMem() : ERREUR ---> pointeur NULL passe en argument #3\n");
        return( NULL );
    };
    /*..................................................*/
    /* lien a / creation de la zone de memoire partagee */
    /*..................................................*/
    if(( *iFd = shm_open(szAreaName, O_RDWR, 0600)) < 0 )
    {
        if( iCreate > 0 )
        {
            if(( *iFd = shm_open(szAreaName, O_RDWR | O_CREAT, 0600)) < 0)
            {
                fprintf(stderr,"Link2SharedMem() :  ERREUR ---> appel a shm_open() pour creation \n");
                fprintf(stderr,"                    code = %d (%s)\n", errno, (char *)(strerror(errno)));
                return( NULL );
            };
        }
        else
        {
            fprintf(stderr,"Link2SharedMem() :  ERREUR ---> appel a shm_open() pour lien \n");
            fprintf(stderr,"                    code = %d (%s)\n", errno, (char *)(strerror(errno)));
            return( NULL );
        };
    };
    /* ajustement de la taille (en octets) */
    if( ftruncate(*iFd, iSize) < 0 )
    {
        fprintf(stderr,"Link2SharedMem() :  ERREUR ---> appel a ftruncate() \n");
        fprintf(stderr,"                    code = %d (%s)\n", errno, (char *)(strerror(errno)));
        return( NULL );
    };
    /* mapping dans la memoire du processus */
     if((vAddr = mmap(NULL, iSize, PROT_READ | PROT_WRITE, MAP_SHARED, *iFd, 0)) == MAP_FAILED )
    {
        fprintf(stderr,"Link2SharedMem() :  ERREUR ---> appel a mmap() #1\n");
        fprintf(stderr,"                    code = %d (%s)\n", errno, (char *)(strerror(errno)));
        return( NULL );
    };
    /* fini */
    return( vAddr );
}
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/* mise a jour des vitesses au centre cinematique */
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
void updateVelocity( void )
{
    double wL;      /* ->valeur courante de la vitesse angulaire roue gauche */
    double wR;      /* ->valeur courante de la vitesse angulaire roue droite */
    double Vc;      /* ->vitesse lineaire au centre cinematique              */
    double Wc;      /* ->vitesse angulaire au centre cinematique             */
    /* photo.. */
    wL  = *lpdb_wL;
    wR  = *lpdb_wR;
    /* calcul */
    Vc = 0.5 * R0 * (wL + wR);
    Wc = (R0/w) * (wR - wL);
    /*_________________________________________________________________*/

#if defined(USR_DBG)²
    if( (iCount % REFRESH_RATE) == 0 )
    {
        printf("Vc = %lf Wc = %lf \n", Vc, Wc);
    };
     iCount++;
#endif

    /*_________________________________________________________________*/
    /* mise a jour */
    lpdb_vel[OFFSET_VC] = Vc;
    lpdb_vel[OFFSET_WC] = Wc;
}
/*&&&&&&&&&&&&&&&&&&&&&&&&*/
/* gestionnaire de signal */
/*&&&&&&&&&&&&&&&&&&&&&&&&*/
void SignalHandler( int signal )
{
    if( signal == SIGALRM)
    {
        updateVelocity();
    };
}
/*######*/
/* MAIN */
/*######*/
int main( int argc, char *argv[])
{
    int     iFdStateL;                      /* ->descripteur pour la zone d'etat moteur gauche                */
    int     iFdStateR;                      /* ->descripteur pour la zone d'etat moteur droit                 */
    int     iFdVelocity;                    /* ->descripteur pour la zone des vitesses                        */
    int     iLoops = 0;                     /* ->compte le nombre d'iterations effectuees                     */
    double  *lpdb_stateL;                   /* ->pointeur sur la zone partagee contenant l'etat moteur gauche */
    double  *lpdb_stateR;                   /* ->pointeur sur la zone partagee contenant l'etat moteur droit  */
    struct sigaction    sa;                 /* ->gestion du signal handler                                    */
    struct sigaction    sa_old;             /* ->gestion du signal handler                                    */
    sigset_t            mask;               /* ->liste des signaux a masquer                                  */
    struct itimerval    sTime;              /* ->periode du timer                                             */
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


   
    if( sscanf(argv[1],"%lf",&R0) == 0 )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> l'argument #1 doit etre reel\n", argv[0]);
        usage(argv[0]);
        return( 0 );
    };
    if( sscanf(argv[2],"%lf",&w) == 0 )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> l'argument #2 doit etre reel\n", argv[0]);
        usage(argv[0]);
        return( 0 );
    };
    if( sscanf(argv[3],"%lf",&Te) == 0 )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> l'argument #3 doit etre reel\n", argv[0]);
        usage(argv[0]);
        return( 0 );
    };

    /*................................................*/
    /* lien / creation aux zones de memoire partagees */
    /*................................................*/

    if( (lpdb_stateL = (double *)(Link2SharedMem(STATE_L, 2 * sizeof(double), &iFdStateL, 1 ))) == NULL )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> appel a Link2SharedMem() #1\n", argv[0]);
        return( 0 );
    };
    if( (lpdb_stateR = (double *)(Link2SharedMem(STATE_R, 2 * sizeof(double), &iFdStateR, 1 ))) == NULL )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> appel a Link2SharedMem() #2\n", argv[0]);
        return( 0 );
    };
    if( (lpdb_vel = (double *)(Link2SharedMem(VELOCITY, 2 * sizeof(double), &iFdVelocity, 1 ))) == NULL )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> appel a Link2SharedMem() #3\n", argv[0]);
        return( 0 );
    };

    lpdb_wL  = &lpdb_stateL[OFFSET_W];
    lpdb_wR  = &lpdb_stateR[OFFSET_W];
    /*.................*/
    /* initialisations */
    /*.................*/
    /*............................................*/
    /* installation de la routine d'interception  */
    /*............................................*/
    memset(&sa,0,sizeof(struct sigaction));
    sigemptyset( &mask );
    sa.sa_mask = mask;
    sa.sa_handler = SignalHandler;
    sa.sa_flags = 0;
    if( sigaction(SIGALRM, &sa, &sa_old) < 0 )
    {
        fprintf(stderr,"%s.main() :  ERREUR ---> appel a sigaction() #1\n", argv[0]);
        fprintf(stderr,"             code = %d (%s)\n", errno, (char *)(strerror(errno)));
        exit( -errno );
    };
    /*........................*/
    /* configuration du timer */
    /*........................*/
    sTime.it_interval.tv_sec = (int)(Te);
    sTime.it_interval.tv_usec = (int)((Te - (int)(Te))*1e6);
    
    sTime.it_value.tv_sec = (int)(Te);
    sTime.it_value.tv_usec = (int)((Te - (int)(Te))*1e6);
    
    if( setitimer( ITIMER_REAL, &sTime, NULL) < 0 )
     {
        fprintf(stderr,"%s.main() :  ERREUR ---> appel a setitimer() \n", argv[0]);
        fprintf(stderr,"             code = %d (%s)\n", errno, (char *)(strerror(errno)));
        exit( -errno );
    };
    /*************************/
    /* fonctionnement normal */
    /*************************/
    while( GoOn)
    {
        pause();
        /*___________________________________________________________________*/
#if defined(USR_DBG)
        if( (iLoops % (int)(REFRESH_RATE)) == 0)
        {
           printf("Vc = %lf Wc = %lf \n", lpdb_vel[0], lpdb_vel[1]); 
        };
        iLoops++;
#endif
        /*___________________________________________________________________*/
    }
    return( 0 );   
}
