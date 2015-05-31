/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2014 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

/*******************************************************************************
 * Process pixel stage 1
 ******************************************************************************/

float decodePixelMeasurement(global const ushort *data, global const short *lut11to16, const uint sub, const uint x, const uint y)
{
    uint row_idx = (424 * sub + (y < 212 ? y + 212 : 423 - y)) * 352;
    uint idx = (((x >> 2) + ((x << 7) & BFI_BITMASK)) * 11) & (uint)0xffffffff;

    uint col_idx = idx >> 4;
    uint upper_bytes = idx & 15;
    uint lower_bytes = 16 - upper_bytes;

    uint data_idx0 = row_idx + col_idx;
    uint data_idx1 = row_idx + col_idx + 1;

    return (float)lut11to16[(x < 1 || 510 < x || col_idx > 352) ? 0 : ((data[data_idx0] >> upper_bytes) | (data[data_idx1] << lower_bytes)) & 2047];
}

float2 processMeasurementTriple(const float ab_multiplier_per_frq, const float p0, const float3 v, int *invalid)
{
    float3 p0vec = (float3)(p0 + PHASE_IN_RAD0, p0 + PHASE_IN_RAD1, p0 + PHASE_IN_RAD2);
    float3 p0cos = cos(p0vec);
    float3 p0sin = sin(-p0vec);

    *invalid = *invalid && any(isequal(v, (float3)(32767.0f)));

    return (float2)(dot(v, p0cos), dot(v, p0sin)) * ab_multiplier_per_frq;
}

void kernel processPixelStage1(global const short *lut11to16, global const float *z_table, global const float3 *p0_table, global const ushort *data,
                               global float3 *a_out, global float3 *b_out, global float3 *n_out, global float *ir_out)
{
    const uint i = get_global_id(0);

    const uint x = i % 512;
    const uint y = i / 512;

    const uint y_in = (423 - y);

    const float zmultiplier = z_table[i];
    int valid = (int)(0.0f < zmultiplier);
    int saturatedX = valid;
    int saturatedY = valid;
    int saturatedZ = valid;
    int3 invalid_pixel = (int3)((int)(!valid));
    const float3 p0 = p0_table[i];

    const float3 v0 = (float3)(decodePixelMeasurement(data, lut11to16, 0, x, y_in),
			       decodePixelMeasurement(data, lut11to16, 1, x, y_in),
			       decodePixelMeasurement(data, lut11to16, 2, x, y_in));
    const float2 ab0 = processMeasurementTriple(AB_MULTIPLIER_PER_FRQ0, p0.x, v0, &saturatedX);

    const float3 v1 = (float3)(decodePixelMeasurement(data, lut11to16, 3, x, y_in),
			       decodePixelMeasurement(data, lut11to16, 4, x, y_in),
			       decodePixelMeasurement(data, lut11to16, 5, x, y_in));
    const float2 ab1 = processMeasurementTriple(AB_MULTIPLIER_PER_FRQ1, p0.y, v1, &saturatedY);

    const float3 v2 = (float3)(decodePixelMeasurement(data, lut11to16, 6, x, y_in),
			       decodePixelMeasurement(data, lut11to16, 7, x, y_in),
			       decodePixelMeasurement(data, lut11to16, 8, x, y_in));
    const float2 ab2 = processMeasurementTriple(AB_MULTIPLIER_PER_FRQ2, p0.z, v2, &saturatedZ);

    float3 a = select((float3)(ab0.x, ab1.x, ab2.x), (float3)(0.0f), invalid_pixel);
    float3 b = select((float3)(ab0.y, ab1.y, ab2.y), (float3)(0.0f), invalid_pixel);
    float3 n = sqrt(a * a + b * b);

    int3 saturated = (int3)(saturatedX, saturatedY, saturatedZ);
    a = select(a, (float3)(0.0f), saturated);
    b = select(b, (float3)(0.0f), saturated);

    a_out[i] = a;
    b_out[i] = b;
    n_out[i] = n;
    ir_out[i] = min(dot(select(n, (float3)(65535.0f), saturated), (float3)(0.333333333f  * AB_MULTIPLIER * AB_OUTPUT_MULTIPLIER)), 65535.0f);
}


/*******************************************************************************
 * Process pixel stage 2
 ******************************************************************************/
void kernel processPixelStage2(global const float3 *a_in, global const float3 *b_in, global const float *x_table, global const float *z_table,
                               global float *depth, global float *ir_sums)
{
    const uint i = get_global_id(0);
    float3 a = a_in[i];
    float3 b = b_in[i];

    float3 phase = atan2(b, a);
    phase = select(phase, phase + 2.0f * M_PI_F, isless(phase, (float3)(0.0f)));
    phase = select(phase, (float3)(0.0f), isnan(phase));
    float3 ir = sqrt(a * a + b * b) * AB_MULTIPLIER;

    float ir_sum = ir.x + ir.y + ir.z;
    float ir_min = min(ir.x, min(ir.y, ir.z));
    float ir_max = max(ir.x, max(ir.y, ir.z));

    float phase_final = 0;

    if(ir_min >= INDIVIDUAL_AB_THRESHOLD && ir_sum >= AB_THRESHOLD)
    {
	float3 t = phase / (2.0f * M_PI_F) * (float3)(3.0f, 15.0f, 2.0f);

	float t0 = t.x;
	float t1 = t.y;
	float t2 = t.z;

	float t5 = (floor((t1 - t0) * 0.333333f + 0.5f) * 3.0f + t0);
	float t3 = (-t2 + t5);
	float t4 = t3 * 2.0f;

	bool c1 = t4 >= -t4; // true if t4 positive

	float f1 = c1 ? 2.0f : -2.0f;
	float f2 = c1 ? 0.5f : -0.5f;
	t3 *= f2;
	t3 = (t3 - floor(t3)) * f1;

	bool c2 = 0.5f < fabs(t3) && fabs(t3) < 1.5f;

	float t6 = c2 ? t5 + 15.0f : t5;
	float t7 = c2 ? t1 + 15.0f : t1;

	float t8 = (floor((-t2 + t6) * 0.5f + 0.5f) * 2.0f + t2) * 0.5f;

	t6 *= 0.333333f; // = / 3
	t7 *= 0.066667f; // = / 15

	float t9 = (t8 + t6 + t7); // transformed phase measurements (they are transformed and divided by the values the original values were multiplied with)
	float t10 = t9 * 0.333333f; // some avg

	t6 *= 2.0f * M_PI_F;
	t7 *= 2.0f * M_PI_F;
	t8 *= 2.0f * M_PI_F;

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

	bool slope_positive = 0 < AB_CONFIDENCE_SLOPE;

	float ir_x = slope_positive ? ir_min : ir_max;

	ir_x = log(ir_x);
	ir_x = (ir_x * AB_CONFIDENCE_SLOPE * 0.301030f + AB_CONFIDENCE_OFFSET) * 3.321928f;
	ir_x = exp(ir_x);
	ir_x = clamp(ir_x, MIN_DEALIAS_CONFIDENCE, MAX_DEALIAS_CONFIDENCE);
	ir_x *= ir_x;

	float mask2 = ir_x >= norm ? 1.0f : 0.0f;

	float t11 = t10 * mask2;

	float mask3 = MAX_DEALIAS_CONFIDENCE * MAX_DEALIAS_CONFIDENCE >= norm ? 1.0f : 0.0f;
	t10 *= mask3;
	phase_final = true/*(modeMask & 2) != 0*/ ? t11 : t10;
    }

    float zmultiplier = z_table[i];
    float xmultiplier = x_table[i];

    phase_final = 0.0f < phase_final ? phase_final + PHASE_OFFSET : phase_final;

    float depth_linear = zmultiplier * phase_final;
    float max_depth = phase_final * UNAMBIGIOUS_DIST * 2.0;

    bool cond1 = /*(modeMask & 32) != 0*/ true && 0.0f < depth_linear && 0.0f < max_depth;

    xmultiplier = (xmultiplier * 90.0) / (max_depth * max_depth * 8192.0);

    float depth_fit = depth_linear / (-depth_linear * xmultiplier + 1);
    depth_fit = depth_fit < 0.0f ? 0.0f : depth_fit;

    float d = cond1 ? depth_fit : depth_linear; // r1.y -> later r2.z
    depth[i] = d;
    ir_sums[i] = ir_sum;
}

