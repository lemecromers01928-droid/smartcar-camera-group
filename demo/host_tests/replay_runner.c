#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "img_process.h"

static int read_token(FILE *file, char *buffer, size_t capacity)
{
    int ch;
    size_t used = 0;

    do
    {
        ch = fgetc(file);
        if (ch == '#')
        {
            while (ch != '\n' && ch != EOF)
                ch = fgetc(file);
        }
    } while (ch != EOF && isspace((unsigned char)ch));

    if (ch == EOF)
        return 0;
    while (ch != EOF && !isspace((unsigned char)ch))
    {
        if (used + 1U >= capacity)
            return 0;
        buffer[used++] = (char)ch;
        ch = fgetc(file);
    }
    buffer[used] = '\0';
    return 1;
}

static uint8_t *load_pgm(const char *path, int *width, int *height)
{
    FILE *file = fopen(path, "rb");
    char token[64];
    int binary;
    int max_value;
    size_t count;
    uint8_t *pixels;

    if (file == NULL || !read_token(file, token, sizeof(token)))
        return NULL;
    binary = strcmp(token, "P5") == 0;
    if (!binary && strcmp(token, "P2") != 0)
    {
        fclose(file);
        return NULL;
    }
    if (!read_token(file, token, sizeof(token)))
        goto fail;
    *width = atoi(token);
    if (!read_token(file, token, sizeof(token)))
        goto fail;
    *height = atoi(token);
    if (!read_token(file, token, sizeof(token)))
        goto fail;
    max_value = atoi(token);
    if (*width <= 0 || *width > 255 || *height <= 0 || *height > 255 || max_value <= 0 || max_value > 255)
        goto fail;

    count = (size_t)(*width) * (size_t)(*height);
    pixels = (uint8_t *)malloc(count);
    if (pixels == NULL)
        goto fail;
    if (binary)
    {
        if (fread(pixels, 1U, count, file) != count)
        {
            free(pixels);
            goto fail;
        }
    }
    else
    {
        size_t index;
        for (index = 0; index < count; ++index)
        {
            if (!read_token(file, token, sizeof(token)))
            {
                free(pixels);
                goto fail;
            }
            pixels[index] = (uint8_t)atoi(token);
        }
    }
    fclose(file);
    return pixels;

fail:
    fclose(file);
    return NULL;
}

int main(int argc, char **argv)
{
    int width;
    int height;
    int threshold_offset;
    int row_from_bottom;
    int threshold_value;
    int sample_row;
    uint8_t *pixels;
    short histogram[256];
    uint8_t *left;
    uint8_t *mid;
    uint8_t *right;
    uint8_t valid;

    if (argc != 4)
    {
        fprintf(stderr, "usage: replay_runner image.pgm threshold_offset row_from_bottom\n");
        return 2;
    }
    pixels = load_pgm(argv[1], &width, &height);
    if (pixels == NULL)
    {
        fprintf(stderr, "failed to read P2/P5 PGM\n");
        return 2;
    }
    threshold_offset = atoi(argv[2]);
    row_from_bottom = atoi(argv[3]);
    left = (uint8_t *)calloc((size_t)height, 1U);
    mid = (uint8_t *)calloc((size_t)height, 1U);
    right = (uint8_t *)calloc((size_t)height, 1U);
    if (left == NULL || mid == NULL || right == NULL)
    {
        fprintf(stderr, "allocation failed\n");
        free(pixels);
        free(left);
        free(mid);
        free(right);
        return 2;
    }

    get_hist_gram(pixels, (unsigned short)height, (unsigned short)width, histogram);
    threshold_value = (int)get_threshold_otsu(histogram) + threshold_offset;
    if (threshold_value < 0)
        threshold_value = 0;
    if (threshold_value > 255)
        threshold_value = 255;
    binaryzation_process(pixels, (unsigned short)height, (unsigned short)width,
                          (unsigned int)threshold_value);
    auxiliary_process(pixels, (uint8_t)height, (uint8_t)width,
                      (unsigned char)threshold_value, left, mid, right);

    sample_row = height - row_from_bottom;
    if (sample_row < 0)
        sample_row = 0;
    if (sample_row >= height)
        sample_row = height - 1;
    valid = lane_is_valid_at_row(left, right, (uint8_t)sample_row, (uint8_t)width);
    printf("center=%u,left=%u,right=%u,width=%u,threshold=%d,valid=%u\n",
           (unsigned int)mid[sample_row],
           (unsigned int)left[sample_row],
           (unsigned int)right[sample_row],
           (unsigned int)(right[sample_row] - left[sample_row]),
           threshold_value,
           (unsigned int)valid);

    free(pixels);
    free(left);
    free(mid);
    free(right);
    return valid ? 0 : 1;
}
