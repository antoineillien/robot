/*===============================*/
/* regulation PID d'un moteur CC */
/* ------------------------------*/
/* J.BOONAERT AMSE 2021-2022     */
/*===============================*/
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
#define CMD_BASENAME        "COMMAND_"
#define STATE_BASENAME      "STATE_"
#define TARGET_BASENAME     "TARGET_"
#define NB_ARGS         6               /* ->nombre d'arguments a passer en ligne de commande                            */
#define REFRESH_RATE    5               /* ->nombre d'iterations a realiser pour 1 affichage de l'etat et de la commande */
#define OFFSET_W        0               /* ->offset sur la zone d'etat pour acceder a la vitesse angulaire               */
#define OFFSET_I        1               /* ->offset sur la zone d'etat pour acceder a l'intensite                        */
#define STR_LEN         64              /* ->taille des chaines par defaut                                               */
/*----------*/
/* globales */
/*----------*/
int     GoOn = 1;           /* ->controle d'execution du processus            */
int     iCount = 0;         /* ->comptage du nombre d'iterations              */
double  *lpdb_u;            /* ->pointeur sur la commande                     */
double  *lpdb_w;            /* ->pointeur sur la vitesse angulaire            */
double  *lpdb_Tv;           /* ->pointeur sur la consigne de vitesse          */
double  *lpdb_i;            /* ->pointeur sur le courant                      */
double  Te;                 /* ->periode d'echantillonnage                    */
double  e;                  /* ->erreur courante                              */
double  e_prev;             /* ->erreur passee                                */
double  De;                 /* ->derivee de l'erreur                          */
double  Ie;                 /* ->integrale de l'erreur                        */
double  Ie_prev;            /* ->valeur precedente de l'integrale de l'erreur */
double  Kcoeff;             /* ->action proportionnelle                       */
double  Icoeff;             /* ->action integrale                             */
double  Dcoeff;             /* ->action derivee                               */
/*--------------*/
/* declarations */
/*--------------*/
void usage( char *);        /* ->aide de ce programme                         */
void *Link2SharedMem( char *, int, int *, int);
                            /* ->creation ou lien a une zone de memoire       */
                            /*   partagee                                     */
void updateCommand(void);   /* ->mise a jour de la commande du moteur         */
void SignalHandler(int);    /* ->gestionnaire de signale                      */
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
    printf("%s <P> <I> <D> <Periode d'ech.> <drive>\n", szPgmName);
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
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/* mise a jour de la commande du moteur */
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
void updateCommand( void )
{
    double u;       /* ->valeur courante de la commande          */
    double w;       /* ->valeur courante de la vitesse angulaire */
    double Tv;      /* ->valeur courante de la consigne          */
    double u_new;   /* ->nouvelle valeur de la commande          */
    /* photo.. */
    u  = *lpdb_u;
    w  = *lpdb_w;
    Tv = *lpdb_Tv; 
    /* init */
    e = Tv - w;
    De = (e - e_prev)/Te;
    Ie = Ie_prev + Te * e;
    /* calcul */
    u = Kcoeff * ( e + Icoeff * Ie + Dcoeff * De);
    /*_________________________________________________________________*/
#if defined(USR_DBG)
    if( (iCount % REFRESH_RATE) == 0 )
    {
        printf("Te = %lf e = %lf De = %lf Ie = %lf\n", Te, e, De, Ie);
        printf("u PID = %lf\n", u);
    };
     iCount++;
#endif
    /*_________________________________________________________________*/
    /* mise a jour */
    Ie_prev = Ie;
    e_prev  = e;
    *lpdb_u = u;
}
/*&&&&&&&&&&&&&&&&&&&&&&&&*/
/* gestionnaire de signal */
/*&&&&&&&&&&&&&&&&&&&&&&&&*/
void SignalHandler( int signal )
{
    if( signal == SIGALRM)
    {
        updateCommand();
    };
}
/*######*/
/* MAIN */
/*######*/
int main( int argc, char *argv[])
{
    char    cDriveID;                       /* ->caractere pour identifier le moteur            */
    char    szCmdAreaName[STR_LEN];         /* ->nom de la zone contenant la commande           */
    char    szStateAreaName[STR_LEN];       /* ->nom de la zone contenant l'etat du moteur      */
    char    szTargetAreaName[STR_LEN];      /* ->nom de la zone contenant la consigne           */
    int     iFdCmd;                         /* ->descripteur pour la zone de commande           */
    int     iFdState;                       /* ->descripteur pour la zone d'etat                */
    int     iFdTarget;                      /* ->descripteur pour la zone de consigne           */
    int     iLoops = 0;                     /* ->compte le nombre d'iterations effectuees       */
    double  *lpdb_state;                    /* ->pointeur sur la zone partagee contenant l'etat */
    double  r;                              /* ->resistance de l'induit                         */
    double  l;                              /* ->inductance                                     */
    double  Ke;                             /* ->constante electrique                           */
    double  Km;                             /* ->constante moteur                               */
    double  f;                              /* ->coefficient de frottement                      */
    double  j;                              /* ->inertie totale au rotor                        */
    struct sigaction    sa;                 /* ->gestion du signal handler                      */
    struct sigaction    sa_old;             /* ->gestion du signal handler                      */
    sigset_t            mask;               /* ->liste des signaux a masquer                    */
    struct itimerval    sTime;              /* ->periode du timer                               */
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
    if( sscanf(argv[1],"%lf",&Kcoeff) == 0 )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> l'argument #1 doit etre reel\n", argv[0]);
        usage(argv[0]);
        return( 0 );
    };
    if( sscanf(argv[2],"%lf",&Icoeff) == 0 )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> l'argument #2 doit etre reel\n", argv[0]);
        usage(argv[0]);
        return( 0 );
    };
    if( sscanf(argv[3],"%lf",&Dcoeff) == 0 )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> l'argument #3 doit etre reel\n", argv[0]);
        usage(argv[0]);
        return( 0 );
    };
    if( sscanf(argv[4],"%lf",&Te) == 0 )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> l'argument #4 doit etre reel\n", argv[0]);
        usage(argv[0]);
        return( 0 );
    };
    if( sscanf(argv[5],"%c",&cDriveID) == 0 )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> l'argument #8 doit etre un caractere\n", argv[0]);
        usage(argv[0]);
        return( 0 );
    };
    printf("Kcoeff = %lf\tIcoeff = %lf\tDcoeff = %lf\n", Kcoeff, Icoeff, Dcoeff);
    /*................................................*/
    /* lien / creation aux zones de memoire partagees */
    /*................................................*/
    sprintf(szCmdAreaName,"%s%c", CMD_BASENAME, cDriveID);
    if( (lpdb_u = (double *)(Link2SharedMem(szCmdAreaName, sizeof(double), &iFdCmd, 1 ))) == NULL )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> appel a Link2SharedMem() #1\n", argv[0]);
        return( 0 );
    };
    sprintf(szStateAreaName,"%s%c", STATE_BASENAME, cDriveID);
    if( (lpdb_state = (double *)(Link2SharedMem(szStateAreaName, 2 * sizeof(double), &iFdState, 1 ))) == NULL )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> appel a Link2SharedMem() #2\n", argv[0]);
        return( 0 );
    };
    lpdb_w = &lpdb_state[OFFSET_W];
    lpdb_i = &lpdb_state[OFFSET_I];
    sprintf(szTargetAreaName,"%s%c", TARGET_BASENAME, cDriveID);
    if( (lpdb_Tv = (double *)(Link2SharedMem(szTargetAreaName, sizeof(double), &iFdTarget, 1 ))) == NULL )
    {
        fprintf(stderr,"%s.main()  : ERREUR ---> appel a Link2SharedMem() #3\n", argv[0]);
        return( 0 );
    };
    *lpdb_Tv = 0.0;
    /*.................*/
    /* initialisations */
    /*.................*/
    e       = 0.0;
    e_prev  = 0.0;
    Ie      = 0.0;
    Ie_prev = 0.0;
    De      = 0.0;
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
        /*___________________________________________________*/
#if defined(USR_DBG)
        if( (iLoops % (int)(REFRESH_RATE)) == 0)
        {
            printf("Tv = %lf w = %lf \n", *lpdb_Tv, *lpdb_w);
        };
        iLoops++;
        /*___________________________________________________*/
#endif
    }
    return( 0 );   
}