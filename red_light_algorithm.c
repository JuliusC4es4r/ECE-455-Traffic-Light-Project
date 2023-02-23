#include <stdio.h>
#include <stdint.h>


uint8_t lights = 0b00000000;

void printLights(void)
{
    for(int i = 7; i >= 0; i--)
    {
        int8_t out = lights & (0b1 << i);
        if(out)
        {
            printf("1");
        }
        else
        {
            printf("0");
        }
    }
    printf("\n");
}

int main(void)
{
    printLights();
    for(int i = 0; i < 50; i++)
    {
        uint8_t result, mask, bits_to_shift = 0x00;

        int n = 0;
        for(int j = 7; j >= 0; j--)
        {
            if((lights & (0x01 << j)) == 0)
            {
                n = j;
                break;
            }
        }

        mask = (0x01 << n) - 1; // create mask for n least significant bits
        bits_to_shift = lights & mask; // isolate n least significant bits
        bits_to_shift <<= 1; // shift bits left by n positions
        result = bits_to_shift | (lights & ~mask); // combine shifted bits with n-1 most significant bits

        lights = result;
        printLights();
    }
    return 0;
}
