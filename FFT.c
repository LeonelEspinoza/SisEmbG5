#include <math.h>

#define M_PI 3.14159

/**
 * @brief Funcion que calcula la FFT de un arreglo y guarda el resultado inplace
 *
 * @param array Arreglo de elementos sobre los que se quiere calcular la FFT
 * @param size Tamano del arreglo
 * @param array_re Direccion del arreglo donde se guardara la parte real. Debe ser de tamano size
 * @param array_im Direccion del arreglo donde se guardara la parte imaginaria. Debe ser de tamano size
 */
void calcularFFT(float *array, int size, float *array_re, float *array_im) {
    for (int k = 0; k < size; k++) {
        float real = 0;
        float imag = 0;

        for (int n = 0; n < size; n++) {
            float angulo = 2 * M_PI * k * n / size;
            float cos_angulo = cos(angulo);
            float sin_angulo = -sin(angulo);

            real += array[n] * cos_angulo;
            imag += array[n] * sin_angulo;
        }
        real /= size;
        imag /= size;
        array_re[k] = real;
        array_im[k] = imag;
    }
}
