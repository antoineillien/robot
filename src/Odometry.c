/** le but de ce programme est de récupérer les valeurs Vc et Wc qui sortent de Kinematics pour mettre à jour la position et l'orientation du robot

Voici la forme générale de ce programme : 

#include ...

#define ...

déclaration de variables à partir des espaces mémoires 

usage et erreurs

mise à jour de la position

Gestion des signaux

**/ 

/** 

Concretement, cela veut dire que l'on va aller chercher la valeur de Vc et Wc dans lpbd_vel (avec l'OFFSET de chacun), 
les attribuer dans deux pointeurs lpbd_Vc et lpbd_Wc dans la fonction main, puis utiliser ces valeurs pour faire les calculs de position
et enfin exporter cette position dans une zone mémoire partagée qui sera appelée lpbd_position

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


                                                    
#define OFFSET_VC       0               /* ->offset sur la zone d'etat pour acceder a la vitesse angulaire               */
#define OFFSET_WC       1               /* ->offset sur la zone d'etat pour acceder a l'intensite                        */
#define STR_LEN         64              /* ->taille des chaines par defaut                                               */

#define NB_ARGS         9               /* ->nombre d'arguments a passer en ligne de commande                            */
#define VELOCITY            "VELOCITY"  /* ->nom de la zone partagee pour les vitesses lineaire et angulaire             */
#define REFRESH_RATE    5               /* ->nombre d'iterations a realiser pour 1 affichage de l'etat et de la commande */
#define POSITION            "POSITION"  /* ->nom de la zone partagee pour la position            */

/*----------*/
/* globales --> j'en importe beaucoup, je supprimerai au fur et à mesure */
/*----------*/
int     GoOn = 1;           /* ->controle d'execution du processus             */
int     iCount = 0;         /* ->comptage du nombre d'iterations               */

/*double  *lpdb_vel;           ->pointeur sur la zone partagee des vitesses, déja défini dans Kinematics    */ 
double  Te;                 /* ->periode d'echantillonnage                     */
                        

/*double *lpdb_w;         ->pointeur sur la vitesse angulaire */


/* variable globale que je crée */

/** Inutile
double *lpdb_Xc;
double *lpdb_Yc;
double *lpdb_tehtaC;
**/
double *lpdb_Vc;
double *lpdb_Wc;
double *lpdb_position; /* tableau de 3 double, X, Y , theta  --> il est bati sur le modèle de lpdb_vel*/

/* void usage va ici */
void *Link2SharedMem( char *, int, int *, int);
                            /* ->creation ou lien a une zone de memoire          */
void updatePosition(void); /* cette fonction est construite sur la base de updateVelocity */
void SignalHandler(int);    /* ->gestionnaire de signal, a priori même fonction que dans Kinematics       */

/* je saute la partie usage et erreurs*/

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/* creation ou lien a une zone de memoire */
/* partagee.                              */
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

/* même fonction que dans Kinematics */

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

void updatePosition(void)
{
    double Xc;
    double Yc; 
    double thetaC;

    double Vc;
    double Wc;

    Vc = *lpdb_Vc;
    Wc = *lpdb_Wc; 
    
    /* Calcul */
    Xc = Xc + Vc*cos(thetaC)*Te;
    Yc = Yc + Vc*sin(thetaC)*Te; /* Erreur résolue ici, j'avais un problème de compilation avec sin et cos, il faut ajouter -lm à la fin de la command gcc */
    thetaC = thetaC + Wc*Te;

/** PROBLEME DE COMPILATION : l'exposant ²
    #if defined(USR_DBG)²
    if( (iCount % REFRESH_RATE) == 0 )
    {
        printf("Xc = %lf Yx = %lf thetaC = %lf \n", Xc, Yc, thetaC); /* cette ligne es une simple transposition de la ligne correspondante dans Kinematics 
    };
     iCount++;
#endif 
**/
    /*_________________________________________________________________*/
    /* mise a jour */
    /*     RENDU INUTILE CAR LPDB_POSITION EST UN TABLEAU DE 3 VALEURS
    lpdb_Xc = Xc;
    lpdb_Yc= Yc;
    lpdb_tehtaC = thetaC;
    */
    
    lpdb_position[0] = Xc; /*Ici on met à jour dans la zone partagée de la position */
    lpdb_position[1] = Yc;
    lpdb_position[2] = thetaC; 

}

/*&&&&&&&&&&&&&&&&&&&&&&&&*/
/* gestionnaire de signal */
/*&&&&&&&&&&&&&&&&&&&&&&&&*/
void SignalHandler( int signal )
{
    if( signal == SIGALRM)
    {
        updatePosition();
    };
}
 
/* je copie-colle la fonction main de Kinematics et je la modifie */ 


/*######*/
/* MAIN */
/*######*/
int main( int argc, char *argv[])
{
    int     iFdVc;                      /* ->descripteur pour la zone de Xc              */
    int     iFdWc;                      /* ->descripteur pour la zone de Yc               */

    int     iFdPosition;                    /* ->descripteur pour la zone des vitesses     --> je conserve telle quelle                     */
   
    int     iLoops = 0;                     /* ->compte le nombre d'iterations effectuees                     */
    double  *lpdb_vel;                   /* ->pointeur sur la zone partagee contenant la vitesse, c'est ce qui est calculé par KInematics */
    struct sigaction    sa;                 /* ->gestion du signal handler                                    */
    struct sigaction    sa_old;             /* ->gestion du signal handler                                    */
    sigset_t            mask;               /* ->liste des signaux a masquer                                  */
    struct itimerval    sTime;              /* ->periode du timer                                             */
    /*.......*/
    /* check */
    /*.......*/

    /** neutralisé de manière temporaire
    if( argc != NB_ARGS)
    {
        usage(argv[0]);
        return( 0 );
    };
    **/

    /**
    /*............................
    /* recuperation des arguments 
    /*............................

    Partie neutralisée je n'ai pas eu le temps de travailler sur les conditions des arguments et la fonction usage

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

    Idem sur cette partie, j'ai conscience que c'est problématique de supprimer ces vérifications sur un langage aussi proche de la machine
    je vais essayer d'y revenir


    /*................................................*/
    /* lien / creation aux zones de memoire partagees */ /* à modifier */
    /*................................................
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
    **/
    lpdb_Vc  = &lpdb_vel[OFFSET_VC];
    lpdb_Wc  = &lpdb_vel[OFFSET_WC];
    


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
            printf("Xc = %lf Yx = %lf thetaC = %lf \n", Xc, Yc, thetaC); /* cette ligne es une simple transposition de la ligne correspondante dans Kinematics */
        };
        iLoops++;
#endif
        /*___________________________________________________________________*/
    }
    return( 0 );   
}


/* double *lpdb_Xc;
double *lpdb_Yc;
double *lpdb_tehtaC;
double *lpdb_position[3];  tableau de 3 double, X, Y , theta */
