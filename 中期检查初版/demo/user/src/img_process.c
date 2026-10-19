/*
 *@第十九届东南大学智能车竞赛基础四轮组部分开源学习代码
 */


#include "img_process.h"
#include "string.h"

/*
 *@func: 二值化函数
 */
void binaryzation_process(unsigned char* _img, 
                          const unsigned short _rows, 
                          const unsigned short _cols, 
                          const unsigned int _threshold_value)
{
	unsigned short _size = _rows * _cols;
	for (unsigned short _i = 0; _i < _size; ++_i)
	{
		if (*(_img + _i) > _threshold_value)
			*(_img + _i) = 255;
		else
			*(_img + _i) = 0;
	}
}

/*
 *@func: 获取直方图
 */
void get_hist_gram(unsigned char* _img,
				   const unsigned short _rows, 
				   const unsigned short _cols, 
				   short* _hist_gram)
{
	/* short[256] occupies 512 bytes. Clearing only 256 bytes leaves half of
	 * the previous frame's histogram behind and eventually corrupts Otsu. */
	memset(_hist_gram, 0, sizeof(short) * 256U);
    for (unsigned short _i = 0; _i < _rows; ++_i)
    {
        for (unsigned short _j = 0; _j < _cols; ++_j)
        {
            _hist_gram[_img[_i * _cols + _j]]++;
        }
    }
}

/*
 *@func: 大津法求阈值
 */
unsigned char get_threshold_otsu(const short* _hist_gram)
{
    int X, Y, amount = 0;
    int pixel_back = 0, pixel_fore = 0, pixel_integral_back = 0, pixel_integral_fore = 0, pixel_integral = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    int pixel_min_value, pixel_max_value;
    int threshold = 0;

    for (pixel_min_value = 0; pixel_min_value < 256 && _hist_gram[pixel_min_value] == 0; pixel_min_value++)
        ;
    for (pixel_max_value = 255; pixel_max_value > pixel_min_value && _hist_gram[pixel_max_value] == 0; pixel_max_value--)
        ;
    if (pixel_max_value == pixel_min_value)
        return pixel_max_value; // 图像中只有一个颜色
    if (pixel_min_value + 1 == pixel_max_value)
        return pixel_min_value; // 图像中只有二个颜色

    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        amount += _hist_gram[Y]; //  像素总数

    pixel_integral = 0;
    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        pixel_integral += _hist_gram[Y] * Y;
    SigmaB = -1;
    for (Y = pixel_min_value; Y < pixel_max_value; Y++)
    {
        pixel_back = pixel_back + _hist_gram[Y];
        pixel_fore = amount - pixel_back;
        OmegaBack = (double)pixel_back / amount;
        OmegaFore = (double)pixel_fore / amount;
        pixel_integral_back += _hist_gram[Y] * Y;
        pixel_integral_fore = pixel_integral - pixel_integral_back;
        MicroBack = (double)pixel_integral_back / pixel_back;
        MicroFore = (double)pixel_integral_fore / pixel_fore;
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            threshold = Y;
        }
    }
    return threshold;
}

/*
 *@func: 边界线寻找（按"贴着画面边框的暗区"找边线）
 *
 * 原来是从画面中心往两边找暗点，赛道上任何一块黑色装饰（斑马线、胶带、
 * 阴影）都会被当成边线，中线立刻被拉过去——这也是赛道上那处黑条区域
 * 判断不稳的原因。现在改成从画面左右边框往里扫：只有一直贴到边框的暗区
 * 才算"赛道外"，画面中间的暗色块完全不参与判断。
 */
void auxiliary_process(uint8_t* _src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols, 
					   unsigned char _threshold_val, 
					   uint8_t* _left_line, uint8_t* _mid_line, uint8_t* _right_line)
{
	/* 传进来的是已经二值化的图，用固定的 0/255 判据，不再用 Otsu 的原始值 */
	(void)_threshold_val;

	for (uint8_t row = 0U; row < _src_rows; row ++)
	{
		const uint8_t* line = _src_pixel_mat + (uint16_t)row * _src_cols;
		uint8_t left  = 0U;                       /* 0   = 没找到（白区一直延伸到画面外） */
		uint8_t right = (uint8_t)(_src_cols - 1U); /* 末列 = 没找到 */
		uint8_t run;
		uint8_t col;

		/* ---- 左边：从最左列往里数连续暗像素，暗区必须贴着边框才算数 ---- */
		run = 0U;
		while ((run < _src_cols) && (line[run] < 128U))
		{
			run ++;
		}
		if ((run >= 2U) && (run < _src_cols))
		{
			/* 再找连续 3 个亮点，避开暗区里的孤立噪点 */
			col = run;
			while (((uint16_t)col + 2U) < _src_cols)
			{
				if ((line[col] >= 128U) && (line[col + 1U] >= 128U) && (line[col + 2U] >= 128U))
				{
					break;
				}
				col ++;
			}
			left = (((uint16_t)col + 2U) < _src_cols) ? col : 0U;
		}

		/* ---- 右边：从最右列往里数连续暗像素 ---- */
		run = 0U;
		while ((run < _src_cols) && (line[_src_cols - 1U - run] < 128U))
		{
			run ++;
		}
		if ((run >= 2U) && (run < _src_cols))
		{
			col = (uint8_t)(_src_cols - 1U - run);
			while (col > 2U)
			{
				if ((line[col] >= 128U) && (line[col - 1U] >= 128U) && (line[col - 2U] >= 128U))
				{
					break;
				}
				col --;
			}
			right = (col > 2U) ? col : (uint8_t)(_src_cols - 1U);
		}

		_left_line[row]  = left;
		_right_line[row] = right;
		_mid_line[row]   = (uint8_t)(((uint16_t)left + (uint16_t)right) >> 1);
	}
}

uint8_t lane_is_valid_at_row(const uint8_t* _left_line,
						 const uint8_t* _right_line,
						 uint8_t _row,
						 uint8_t _cols)
{
	uint8_t left = _left_line[_row];
	uint8_t right = _right_line[_row];
	uint8_t width;

	if (left == 0U || right >= (uint8_t)(_cols - 1U) || right <= left)
		return 0U;
	width = (uint8_t)(right - left);
	return (uint8_t)(width >= 8U && width <= (uint8_t)(_cols - 4U));
}

uint8_t lane_center_at_row(const uint8_t* _left_line,
						 const uint8_t* _right_line,
						 uint8_t _row,
						 uint8_t _cols,
						 uint8_t* _mid,
						 uint16_t* _width_ref)
{
	uint8_t left        = _left_line[_row];
	uint8_t right       = _right_line[_row];
	uint8_t found_left  = (uint8_t)(left  != 0U);
	uint8_t found_right = (uint8_t)(right < (uint8_t)(_cols - 1U));
	uint16_t width      = *_width_ref;
	uint8_t mid;
	uint16_t tmp;

	/* 两个"找到了"但几乎重合，说明整行同色（车压到蓝地胶上时，
	 * 扫描从画面中心立刻命中黑点，left 和 right 都会等于中心列）。
	 * 这种必须判成 LANE_NONE，否则会补出一条假中线，车永远不停。 */
	if ((found_left != 0U) && (found_right != 0U) && ((uint8_t)(right - left) < 8U))
	{
		*_mid = (uint8_t)(_cols >> 1);
		return LANE_NONE;
	}

	if ((found_left != 0U) && (found_right != 0U))
	{
		/* 两边都在：用本行宽度平滑更新参考带宽（3/4 旧 + 1/4 新） */
		width = (uint16_t)((((uint32_t)width * 3U) + (uint32_t)(right - left)) >> 2);
		*_width_ref = width;
		mid = (uint8_t)(((uint16_t)left + (uint16_t)right) >> 1);
	}
	else if (found_left != 0U)
	{
		tmp = (uint16_t)left + (width >> 1);
		mid = (uint8_t)((tmp < _cols) ? tmp : (uint8_t)(_cols - 1U));
	}
	else if (found_right != 0U)
	{
		tmp = width >> 1;
		mid = (uint8_t)((right > tmp) ? (uint8_t)(right - tmp) : 0U);
	}
	else
	{
		*_mid = (uint8_t)(_cols >> 1);
		return LANE_NONE;
	}

	*_mid = mid;
	if ((found_left != 0U) && (found_right != 0U))
		return LANE_BOTH;
	return (found_left != 0U) ? LANE_LEFT_ONLY : LANE_RIGHT_ONLY;
}

