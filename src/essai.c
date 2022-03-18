#import <stdio.h>
#include <unistd.h>
main()
{
printf("Entrer deux nombres entiers:\n") ;
double A =1;
double B =0;
scanf("%d %d",&A,&B);
if(A>B)
	printf("%d est plus grand que %d\n",A,B);
else
	if(A<B)
		printf("%d est plus petit que %d\n",A,B);
	else
		printf("%d est égal à %d\n",A,B);
}
