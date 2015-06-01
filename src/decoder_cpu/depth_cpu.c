/**
 * @file   decoder_depth_cpu.c
 * @author Hiromasa YOSHIMOTO
 * @date   Wed May 13 17:43:51 2015
 * 
 * @brief  
 * 
 * 
 */
#include "module.h"
#include <stdint.h>
#include <math.h>

#include <assert.h>

#pragma GCC optimize ("O3")

typedef struct {
    struct k4w2_decoder_ctx decoder; 
    struct parameters {
	float ab_multiplier;
	float ab_multiplier_per_frq[3];
	float ab_output_multiplier;

	float phase_in_rad[3];

	float phase_offset;
	float unambigious_dist;
	float individual_ab_threshold;
	float ab_threshold;
	float ab_confidence_slope;
	float ab_confidence_offset;
	float min_dealias_confidence;
	float max_dealias_confidence;

//	float min_depth;
//	float max_depth;
    } params;

    float trig_table0[512*424][6];
    float trig_table1[512*424][6];
    float trig_table2[512*424][6];
    float x_table[512*424];
    float z_table[512*424];

    /* work area; work[ctx->num_slot][ 512*424*sizeof(float) * 9 ] */
    unsigned char **work;

} decoder_depth;

static int16_t lut11to16[2048];

#define MIN(a,b)  (a)>(b)?(b):(a)
#define MAX(a,b)  (a)>(b)?(a):(b)

static void
set_params(struct parameters *p)
{
    p->ab_multiplier = 0.6666667f;
    p->ab_multiplier_per_frq[0] = 1.322581f;
    p->ab_multiplier_per_frq[1] = 1.0f;
    p->ab_multiplier_per_frq[2] = 1.612903f;
    p->ab_output_multiplier = 16.0f;

    p->phase_in_rad[0] = 0.0f;
    p->phase_in_rad[1] = 2.094395f;
    p->phase_in_rad[2] = 4.18879f;

    p->phase_offset = 0.0f;
    p->unambigious_dist = 2083.333f;
    p->individual_ab_threshold  = 3.0f;
    p->ab_threshold = 10.0f;
    p->ab_confidence_slope = -0.5330578f;
    p->ab_confidence_offset = 0.7694894f;
    p->min_dealias_confidence = 0.3490659f;
    p->max_dealias_confidence = 0.6108653f;

//    p->min_depth = 500.0f;
//    p->max_depth = 4500.0f;
}

static inline int
bfi(int width, int offset, int src2, int src3)
{
    const int bitmask = (((1 << width)-1) << offset) & 0xffffffff;
    return ((src2 << offset) & bitmask) | (src3 & ~bitmask);
}

static inline int32_t
decodePixelMeasurement(const unsigned char* data, int sub, int x, int y) 
{
    const uint16_t *ptr = (const uint16_t*)(data + KINECT2_DEPTH_FRAME_SIZE * sub);

    const int i = y < 212 ? y + 212 : 423 - y;
    ptr += 352*i;

    /**
       r1.yz = r2.xxyx < l(0, 1, 0, 0) // ilt
       r1.y = r1.z | r1.y // or
    */
    int r1y = x < 1 || y < 0;
    /*
      r1.zw = l(0, 0, 510, 423) < r2.xxxy // ilt
      r1.z = r1.w | r1.z // or
    */
    int r1z = 510 < x || 423 < y;
    /*
      r1.y = r1.z | r1.y // or
    */
    r1y = r1y || r1z;
    /*
      r1.y = r1.y & l(0x1fffffff) // and
    */
    int r1yi = r1y ? 0xffffffff : 0x0;
    r1yi &= 0x1fffffff;

    /*
      bfi r1.z, l(2), l(7), r2.x, l(0)
      ushr r1.w, r2.x, l(2)
      r1.z = r1.w + r1.z // iadd
    */
    int r1zi = bfi(2, 7, x, 0);
    int r1wi = x >> 2;
    r1zi = r1wi + r1zi;

    /*
      imul null, r1.z, r1.z, l(11)
      ushr r1.w, r1.z, l(4)
      r1.y = r1.w + r1.y // iadd
      r1.w = r1.y + l(1) // iadd
      r1.z = r1.z & l(15) // and
      r4.w = -r1.z + l(16) // iadd
    */
    r1zi = (r1zi * 11L) & 0xffffffff;
    r1wi = r1zi >> 4;
    r1yi = r1yi + r1wi;
    r1zi = r1zi & 15;
    int r4wi = -r1zi + 16;

    if(r1yi > 352)
    {
	return lut11to16[0];
    }

    int i1 = ptr[r1yi];
    int i2 = ptr[r1yi + 1];
    i1 = i1 >> r1zi;
    i2 = i2 << r4wi;

    return lut11to16[((i1 | i2) & 2047)];
}


static inline void
transformMeasurements(const float m[3], float ab_multiplier,
		      float out[3]) 
{
    float tmp0 = atan2f((m[1]), (m[0]));
    tmp0 = tmp0 < 0 ? tmp0 + M_PI * 2.0f : tmp0;
    tmp0 = (tmp0 != tmp0) ? 0 : tmp0;

    float tmp1 = sqrtf(m[0] * m[0] + m[1] * m[1]) * ab_multiplier;

    out[0] = tmp0; // phase
    out[1] = tmp1; // ir amplitude - (possibly bilateral filtered)
    out[2] = m[2];
}

static inline void
processMeasurementTriple(const float trig_table[512*424][6],
			 const float * z_table,
			 const float abMultiplierPerFrq,
			 const float ab_multiplier,
			 const int x, const int y,
			 const int32_t m[3],
			 float m_out[3]) 
{
    const int offset = y * 512 + x;
    const float cos_tmp0 = trig_table[offset][0];
    const float cos_tmp1 = trig_table[offset][1];
    const float cos_tmp2 = trig_table[offset][2];

    const float sin_negtmp0 = trig_table[offset][3];
    const float sin_negtmp1 = trig_table[offset][4];
    const float sin_negtmp2 = trig_table[offset][5];

    const float zmultiplier = z_table[offset];
    const int cond0 = 0 < zmultiplier;
    const int cond1 = (m[0] == 32767 || m[1] == 32767 || m[2] == 32767) && cond0;

    // formula given in Patent US 8,587,771 B2
    float tmp3 = cos_tmp0 * m[0] + cos_tmp1 * m[1] + cos_tmp2 * m[2];
    float tmp4 = sin_negtmp0 * m[0] + sin_negtmp1 * m[1] + sin_negtmp2 * m[2];

    // only if modeMask & 32 != 0;
    if(1)//(modeMask & 32) != 0)
    {
	tmp3 *= abMultiplierPerFrq;
	tmp4 *= abMultiplierPerFrq;
    }
    float tmp5 = sqrtf(tmp3 * tmp3 + tmp4 * tmp4) * ab_multiplier;

    // invalid pixel because zmultiplier < 0 ??
    tmp3 = cond0 ? tmp3 : 0;
    tmp4 = cond0 ? tmp4 : 0;
    tmp5 = cond0 ? tmp5 : 0;

    // invalid pixel because saturated?
    tmp3 = !cond1 ? tmp3 : 0;
    tmp4 = !cond1 ? tmp4 : 0;
    tmp5 = !cond1 ? tmp5 : 65535.0; // some kind of norm calculated from tmp3 and tmp4

    m_out[0] = tmp3; // ir image a
    m_out[1] = tmp4; // ir image b
    m_out[2] = tmp5; // ir amplitude
}


    
static inline void
processPixelStage2(int x, int y,
		   const struct parameters * params,
		   const float * z_table,
		   const float * x_table,
		   const float m0_in[3],
		   const float m1_in[3],
		   const float m2_in[3],
		   float * const ir_out, float * const depth_out, float * const ir_sum_out) 
{
    const int offset = y * 512 + x;

    //// 10th measurement
    //float m9 = 1; // decodePixelMeasurement(data, 9, x, y);
    //
    //// WTF?
    //bool cond0 = zmultiplier == 0 || (m9 >= 0 && m9 < 32767);
    //m9 = std::max(-m9, m9);
    //// if m9 is positive or pixel is invalid (zmultiplier) we set it to 0 otherwise to its absolute value O.o
    //m9 = cond0 ? 0 : m9;

    float m0[3], m1[3], m2[3];

    transformMeasurements(m0_in, params->ab_multiplier, m0);
    transformMeasurements(m1_in, params->ab_multiplier, m1);
    transformMeasurements(m2_in, params->ab_multiplier, m2);

    float ir_sum = m0[1] + m1[1] + m2[1];

    float phase;
    {
	float ir_min = MIN( MIN(m0[1], m1[1]), m2[1]);

	if (ir_min < params->individual_ab_threshold || ir_sum < params->ab_threshold)
	{
	    phase = 0;
	}
	else
	{
	    float t0 = m0[0] / (2.0f * M_PI) * 3.0f;
	    float t1 = m1[0] / (2.0f * M_PI) * 15.0f;
	    float t2 = m2[0] / (2.0f * M_PI) * 2.0f;

	    float t5 = (floorf((t1 - t0) * 0.333333f + 0.5f) * 3.0f + t0);
	    float t3 = (-t2 + t5);
	    float t4 = t3 * 2.0f;

	    int c1 = t4 >= -t4; // true if t4 positive

	    float f1 = c1 ? 2.0f : -2.0f;
	    float f2 = c1 ? 0.5f : -0.5f;
	    t3 *= f2;
	    t3 = (t3 - floorf(t3)) * f1;

	    int c2 = 0.5f < fabs(t3) && fabs(t3) < 1.5f;

	    float t6 = c2 ? t5 + 15.0f : t5;
	    float t7 = c2 ? t1 + 15.0f : t1;

	    float t8 = (floorf((-t2 + t6) * 0.5f + 0.5f) * 2.0f + t2) * 0.5f;

	    t6 *= 0.333333f; // = / 3
	    t7 *= 0.066667f; // = / 15

	    float t9 = (t8 + t6 + t7); // transformed phase measurements (they are transformed and divided by the values the original values were multiplied with)
	    float t10 = t9 * 0.333333f; // some avg

	    t6 *= 2.0f * M_PI;
	    t7 *= 2.0f * M_PI;
	    t8 *= 2.0f * M_PI;

	    // some cross product
	    float t8_new = t7 * 0.826977f - t8 * 0.110264f;
	    float t6_new = t8 * 0.551318f - t6 * 0.826977f;
	    float t7_new = t6 * 0.110264f - t7 * 0.551318f;

	    t8 = t8_new;
	    t6 = t6_new;
	    t7 = t7_new;

	    float norm = t8 * t8 + t6 * t6 + t7 * t7;
	    float mask = t9 >= 0.0f ? 1.0f : 0.0f;
	    t10 *= mask;

	    int slope_positive = 0 < params->ab_confidence_slope;

	    float ir_min_ = MIN(MIN(m0[1], m1[1]), m2[1]);
	    float ir_max_ = MAX(MAX(m0[1], m1[1]), m2[1]);

	    float ir_x = slope_positive ? ir_min_ : ir_max_;

	    ir_x = logf(ir_x);
	    ir_x = (ir_x * params->ab_confidence_slope * 0.301030f + params->ab_confidence_offset) * 3.321928f;
	    ir_x = expf(ir_x);
	    ir_x = MIN(params->max_dealias_confidence, MIN(params->min_dealias_confidence, ir_x));
	    ir_x *= ir_x;

	    float mask2 = ir_x >= norm ? 1.0f : 0.0f;

	    float t11 = t10 * mask2;

	    float mask3 = params->max_dealias_confidence * params->max_dealias_confidence >= norm ? 1.0f : 0.0f;
	    t10 *= mask3;
	    phase = 1 /*(modeMask & 2) != 0*/ ? t11 : t10;
	}
    }

    // this seems to be the phase to depth mapping :)
    float zmultiplier = z_table[offset];
    float xmultiplier = x_table[offset];

    phase = 0 < phase ? phase + params->phase_offset : phase;

    float depth_linear = zmultiplier * phase;
    float max_depth = phase * params->unambigious_dist * 2;

    int cond1 = /*(modeMask & 32) != 0*/ 1 && 0 < depth_linear && 0 < max_depth;

    xmultiplier = (xmultiplier * 90) / (max_depth * max_depth * 8192.0);

    float depth_fit = depth_linear / (-depth_linear * xmultiplier + 1);

    depth_fit = depth_fit < 0 ? 0 : depth_fit;
    float depth = cond1 ? depth_fit : depth_linear; // r1.y -> later r2.z

    // depth
    *depth_out = depth;
    if(ir_sum_out != 0)
    {
	*ir_sum_out = ir_sum;
    }

    // ir
    //*ir_out = std::min((m1[2]) * ab_output_multiplier, 65535.0f);
    // ir avg
    *ir_out = MIN((m0[2] + m1[2] + m2[2]) * 0.3333333f * params->ab_output_multiplier, 65535.0f);
    //ir_out[0] = std::min(m0[2] * ab_output_multiplier, 65535.0f);
    //ir_out[1] = std::min(m1[2] * ab_output_multiplier, 65535.0f);
    //ir_out[2] = std::min(m2[2] * ab_output_multiplier, 65535.0f);
}

static int
depth_cpu_open(k4w2_decoder_t ctx, unsigned int type)
{
    decoder_depth * d = (decoder_depth *)ctx;

    if (type != K4W2_DECODER_DEPTH)
	goto err;

    d->work = allocate_bufs(ctx->num_slot, 512 * 424 * sizeof(float)*9);
    if (!d->work)
	goto err;

    return K4W2_SUCCESS;
err:
    free_bufs(d->work);
    d->work = 0;
    
    return K4W2_ERROR;
}

static void
fill_trig_tables(const struct parameters *params, const uint16_t *p0table,
		 float trig_table[512*424][6]) 
{
    int x,y;
    for (y=0;y<424;++y) {
	for (x=0;x<512;++x) {
	    float p0 = -0.000031 * M_PI * p0table[(423-y)*512 + x];
	    int i = y*512 + x;

	    float tmp0 = p0 + params->phase_in_rad[0];
	    float tmp1 = p0 + params->phase_in_rad[1];
	    float tmp2 = p0 + params->phase_in_rad[2];

	    trig_table[i][0] = cos(tmp0);
	    trig_table[i][1] = cos(tmp1);
	    trig_table[i][2] = cos(tmp2);

	    trig_table[i][3] = sin(-tmp0);
	    trig_table[i][4] = sin(-tmp1);
	    trig_table[i][5] = sin(-tmp2);
	}
    }
}


static int
depth_cpu_set_params(k4w2_decoder_t ctx, 
		     struct kinect2_color_camera_param * color,
		     struct kinect2_depth_camera_param * depth,
		     struct kinect2_p0table * p0table)
{
    decoder_depth * d = (decoder_depth *)ctx;
    int r;
    size_t actual_size;
    
    static const char *searchpath[] = {
	".",
#if defined K4W2_DATADIR
	K4W2_DATADIR,
#endif
    };
    set_params(&d->params);
    r = k4w2_search_and_load(searchpath, ARRAY_SIZE(searchpath),
			     "11to16.bin", lut11to16, 4096,
			     &actual_size);
    if (K4W2_SUCCESS != r || 4096 != actual_size)
	return r;

    r = k4w2_search_and_load(searchpath, ARRAY_SIZE(searchpath),
			     "xTable.bin", d->x_table, 512*424*4,
			     &actual_size);
    if (K4W2_SUCCESS != r || 512*424*4 != actual_size)
	return r;

    r = k4w2_search_and_load(searchpath, ARRAY_SIZE(searchpath),
			     "zTable.bin", d->z_table, 512*424*4,
			     &actual_size);
    if (K4W2_SUCCESS != r || 512*424*4 != actual_size)
	return r;

    fill_trig_tables(&d->params, p0table->p0table0, d->trig_table0);
    fill_trig_tables(&d->params, p0table->p0table1, d->trig_table1);
    fill_trig_tables(&d->params, p0table->p0table2, d->trig_table2);

    return K4W2_SUCCESS;
}

static int
depth_cpu_request(k4w2_decoder_t ctx, int slot, const void *src, int src_length)
{
    decoder_depth * d = (decoder_depth *)ctx;

    float * work = (float*)d->work[slot];

    int y;
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for(y = 0; y < 424; ++y) {
	int x;
	for(x = 0; x < 512; ++x) {
	    float *p = work + x*9 + y*512*9;

	    int32_t m0_raw[3], m1_raw[3], m2_raw[3];


	    m0_raw[0] = decodePixelMeasurement(src, 0, x, y);
	    m0_raw[1] = decodePixelMeasurement(src, 1, x, y);
	    m0_raw[2] = decodePixelMeasurement(src, 2, x, y);
	    m1_raw[0] = decodePixelMeasurement(src, 3, x, y);
	    m1_raw[1] = decodePixelMeasurement(src, 4, x, y);
	    m1_raw[2] = decodePixelMeasurement(src, 5, x, y);
	    m2_raw[0] = decodePixelMeasurement(src, 6, x, y);
	    m2_raw[1] = decodePixelMeasurement(src, 7, x, y);
	    m2_raw[2] = decodePixelMeasurement(src, 8, x, y);

	    processMeasurementTriple((const float (*)[6])d->trig_table0, d->z_table,
				     d->params.ab_multiplier_per_frq[0], d->params.ab_multiplier, x, y, m0_raw, p+0);
	    processMeasurementTriple((const float (*)[6])d->trig_table1, d->z_table,
				     d->params.ab_multiplier_per_frq[1], d->params.ab_multiplier, x, y, m1_raw, p+3);
	    processMeasurementTriple((const float (*)[6])d->trig_table2, d->z_table,
				     d->params.ab_multiplier_per_frq[2], d->params.ab_multiplier, x, y, m2_raw, p+6);

	}
    }

    return K4W2_SUCCESS;
}

/*
static int
depth_cpu_wait(k4w2_decoder_t ctx, int slot)
{
    decoder_depth * d = (decoder_depth *)ctx;
    return K4W2_SUCCESS;
}
*/

static int
depth_cpu_fetch(k4w2_decoder_t ctx, int slot, void *dst, int dst_length)
{
    decoder_depth * d = (decoder_depth *)ctx;
    const float *work = (float*)d->work[slot];


    float *dst_i = (dst_length == 424*512*sizeof(float)*2)?(float*)dst + 424*512:NULL;
    float *dst_d = (float*)dst;

    int y;
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (y = 0; y < 424; ++y) {
	int x;
	for (x = 0; x < 512; ++x) {
	    const float *p = work + x*9 + y*512*9;
	    processPixelStage2(x, y,
			       &d->params,
			       d->z_table,
			       d->x_table,
			       p + 0, p + 3, p + 6,
			       (dst_i)?dst_i + (423 - y)*512 + x:NULL,
			       (dst_d)?dst_d + (423 - y)*512 + x:NULL,
			       0);
	}
    }

    return K4W2_SUCCESS;
}
static int
depth_cpu_close(k4w2_decoder_t ctx)
{
    decoder_depth * d = (decoder_depth *)ctx;
    free_bufs(d->work);
    d->work = 0;
    return K4W2_SUCCESS;
}

static const k4w2_decoder_ops ops = {
    .open	= depth_cpu_open,
    .set_params = depth_cpu_set_params,
    .request	= depth_cpu_request,
/*    .wait	= depth_cpu_wait,*/
    .fetch	= depth_cpu_fetch,
    .close	= depth_cpu_close,
};

REGISTER_MODULE(k4w2_decoder_depth_cpu_init)
{
    k4w2_register_decoder("depth cpu", &ops, sizeof(decoder_depth));
}

/*
 * Local Variables:
 * mode: c
 * c-basic-offset:  4
 * End:
 */
