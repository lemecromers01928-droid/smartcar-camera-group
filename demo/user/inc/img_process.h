#ifndef _IMG_PROCESS_H_
#define _IMG_PROCESS_H_
#ifdef RACE_HOST_TEST
#include <stdint.h>
#else
#include "zf_common_headfile.h"
#endif

void get_hist_gram(unsigned char* _img,
				   const unsigned short _rows, 
				   const unsigned short _cols, 
				   short* _hist_gram);
unsigned char get_threshold_otsu(const short* _hist_gram);
void binaryzation_process(unsigned char* _img, 
						  const unsigned short _rows, 
						  const unsigned short _cols, 
						  const unsigned int _threshold_value);
void auxiliary_process(uint8_t* _src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols, 
					   unsigned char _threshold_val, 
					   uint8_t* _left_line, uint8_t* _mid_line, uint8_t* _right_line);
uint8_t lane_is_valid_at_row(const uint8_t* _left_line,
						 const uint8_t* _right_line,
						 uint8_t _row,
						 uint8_t _cols);

/* 采样行的边线状态：宽赛道上经常只有一条边线在画面里，
 * 这时用参考带宽补出另一条，不能直接判"丢线"。 */
typedef enum
{
	LANE_NONE       = 0,        // 两条边线都没扫到
	LANE_BOTH       = 1,        // 两条边线都在画面里
	LANE_LEFT_ONLY  = 2,        // 只扫到左边线，右边靠补线
	LANE_RIGHT_ONLY = 3         // 只扫到右边线，左边靠补线
}lane_state_enum;

uint8_t lane_center_at_row(const uint8_t* _left_line,
						 const uint8_t* _right_line,
						 uint8_t _row,
						 uint8_t _cols,
						 uint8_t* _mid,
						 uint16_t* _width_ref);


#endif
