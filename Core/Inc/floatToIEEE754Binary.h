struct binArray32 {
   char e[32];
};

typedef struct binArray32 Struct32Int;

typedef union {

        float f;
        struct
        {

                /* Order is important.
                 * Here the members of the union data structure
                 *  use the same memory (32 bits).
                 * The ordering is taken
                 * from the LSB to the MSB. */              
                unsigned int byteFour : 8;
                unsigned int byteThree : 8;
		unsigned int byteTwo: 8;
		unsigned int byteOne : 8;

        } raw;
} myfloat32;

void printBinary(int n, int i,int array[]);
void printIEEE(myfloat32 var, int one, int two, int three, int four, int a[], int b[], int c[], int d[], int e[]);
Struct32Int floatToIEEE754Int(float q);