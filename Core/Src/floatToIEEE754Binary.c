/* C program to convert a real value 
  to IEEE 754 floating point representaion */ 

#include <stdio.h> 
#include <stdlib.h>
#include "floatToIEEE754Binary.h"
#include "main.h"

/* or printBinary(int n, int i) */
void printBinary(int n, int i,int array[]) 
{ 

  /* Prints the binary representation 
    of a number n up to i-bits. */
  int k; 
  int j = 0;
  for (k = i - 1; k >= 0; k--) { 
             
    if ((n >> k) & 1) {
            array[j] = 1;
            j++;
    }
    else {
      array[j] = 0;
            j++;
         }
  } 
} 

/* Function to convert real value 
  to IEEE foating point representation */ 

/* printIEEE needs to return a struct with an array of length, l, m, k */
void printIEEE(myfloat32 var, int one, int two, int three, int four, int a[], int b[], int c[], int d[], int e[])
{ 

  int i;

  /* Prints the IEEE 754 representation 
     of a float value (32 bits) */

  printBinary(var.raw.byteOne ,one,a);
  printBinary(var.raw.byteTwo,two,b);
  printBinary(var.raw.byteThree,three,c);
  printBinary(var.raw.byteFour,four,d);

  /* I need to take all of these array elements and roll it into one array */  
  for(i = 0; i < one; i++) {
      e[i] = a[i];
  } 
 
  for(i = 0; i < two; i++) {
      e[i+one] = b[i];
  } 

    for(i = 0; i < three; i++) {
      e[i+one+two] = c[i];
  }

   for(i = 0; i < four; i++) {
      e[i+one+two+three] = d[i];
   }

} 

/* function float To IEEE */
Struct32Int floatToIEEE754Int(float q)
{
    Struct32Int r;
     int *a;
     int *b;
     int *c;
     int *d;
     int *e;
     int one = 8;
     int two = 8;
     int three = 8;
     int four = 8;
     int p, i;

     p = one + two + three + four;

     a = (int *)calloc(one,sizeof(int));
     b = (int *)calloc(two,sizeof(int));
     c = (int *)calloc(three,sizeof(int));
     d = (int *)calloc(four,sizeof(int));
     e = (int *)calloc(p,sizeof(int));

     myfloat32 var;

     var.f = q;

     printIEEE(var,one,two,three,four,a,b,c,d,e);

     /* I need to set r = d */
     for(i = 0; i < p; i++) {
          r.e[i] = e[i];
     }


     free(a);
     free(b);
     free(c);
     free(d);
     free(e);

     /* I need to set r = d... */

     return r;
}


/* Driver Code */
/* I need to move this driver code into its own function */
/*
int main() 
{
       int i;
       float q = -7.75;
       Struct32Int p;
       p = floatToIEEE754Int(q);       
       for(i = 0; i < 32; i++) {
            printf("%d",p.d[i]);
        }

        printf("\n");

	return 0; 
} 
*/
