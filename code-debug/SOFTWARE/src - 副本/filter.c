#include "filter.h"
#include "arm_math.h"
#define M_PI_F 3.1415926 
//init_x  测量初始值，一般为中值
//init_p   后验状态估计值误差的方差的初始值
//q 预测(过程)噪声方差
//r 测量(观测)噪声方差
//以陀螺仪为例，保持陀螺仪不动测试一段时间数据，该数据呈正态分布。
//按3σ原则，取正态分布(3σ)^2作为初始化值

void kalman1_init(kalman1_state *state, float init_x, float init_p)
{
    state->x = init_x;
    state->p = init_p;
    state->A = 1;
    state->H = 1;
    state->q = 1e2;//10e-6;  /* predict noise convariance */
    state->r = 6e2;//10e-5;  /* measure error convariance */
}


float kalman1_filter(kalman1_state *state, float z_measure)
{
    /* Predict */
    state->x = state->A * state->x;
    state->p = state->A * state->A * state->p + state->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement */
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}


void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2])
{
    state->x[0]    = init_x[0];
    state->x[1]    = init_x[1];
    state->p[0][0] = init_p[0][0];
    state->p[0][1] = init_p[0][1];
    state->p[1][0] = init_p[1][0];
    state->p[1][1] = init_p[1][1];
    //state->A       = {{1, 0.1}, {0, 1}};
    state->A[0][0] = 1;
    state->A[0][1] = 0.1;
    state->A[1][0] = 0;
    state->A[1][1] = 1;
    //state->H       = {1,0};
    state->H[0]    = 1;
    state->H[1]    = 0;
    //state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */
    state->q[0]    = 10e-7;
    state->q[1]    = 10e-7;
    state->r       = 10e-7;  /* estimated error convariance */
}


float kalman2_filter(kalman2_state *state, float z_measure)
{
    float temp0 = 0.0f;
    float temp1 = 0.0f;
    float temp = 0.0f;

    /* Step1: Predict */
    state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1];
    state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1];
    /* p(n|n-1)=A^2*p(n-1|n-1)+q */
    state->p[0][0] = state->A[0][0] * state->p[0][0] + state->A[0][1] * state->p[1][0] + state->q[0];
    state->p[0][1] = state->A[0][0] * state->p[0][1] + state->A[1][1] * state->p[1][1];
    state->p[1][0] = state->A[1][0] * state->p[0][0] + state->A[0][1] * state->p[1][0];
    state->p[1][1] = state->A[1][0] * state->p[0][1] + state->A[1][1] * state->p[1][1] + state->q[1];

    /* Step2: Measurement */
    /* gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose. */
    temp0 = state->p[0][0] * state->H[0] + state->p[0][1] * state->H[1];
    temp1 = state->p[1][0] * state->H[0] + state->p[1][1] * state->H[1];
    temp  = state->r + state->H[0] * temp0 + state->H[1] * temp1;
    state->gain[0] = temp0 / temp;
    state->gain[1] = temp1 / temp;
    /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
    temp = state->H[0] * state->x[0] + state->H[1] * state->x[1];
    state->x[0] = state->x[0] + state->gain[0] * (z_measure - temp); 
    state->x[1] = state->x[1] + state->gain[1] * (z_measure - temp);

    /* Update @p: p(n|n) = [I - gain * H] * p(n|n-1) */
    state->p[0][0] = (1 - state->gain[0] * state->H[0]) * state->p[0][0];
    state->p[0][1] = (1 - state->gain[0] * state->H[1]) * state->p[0][1];
    state->p[1][0] = (1 - state->gain[1] * state->H[0]) * state->p[1][0];
    state->p[1][1] = (1 - state->gain[1] * state->H[1]) * state->p[1][1];

    return state->x[0];
}

static float           _cutoff_freq1; 
static float           _a11;
static float           _a21;
static float           _b01;
static float           _b11;
static float           _b21;
static float           _delay_element_11;        // buffered sample -1
static float           _delay_element_21;        // buffered sample -2
void LPF2pSetCutoffFreq_1(float sample_freq, float cutoff_freq)
{
		float fr =0;  
    float ohm =0;
    float c =0;
	
		fr= sample_freq/cutoff_freq;
		ohm=tanf(M_PI_F/fr);
		c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
        //c=1.0f+2.0f*arm_cos_f32(M_PI_F/4.0f)*ohm + ohm*ohm;
        
    _cutoff_freq1 = cutoff_freq;
    if (_cutoff_freq1 > 0.0f) 
		{
				_b01 = ohm*ohm/c;
				_b11 = 2.0f*_b01;
				_b21 = _b01;
				_a11 = 2.0f*(ohm*ohm-1.0f)/c;
				_a21 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
		}
}

float LPF2pApply_1(float sample)
{
	
		float delay_element_0 = 0, output=0;
    if (_cutoff_freq1 <= 0.0f) {
        // no filtering
        return sample;
    }
		else
		{
				delay_element_0 = sample - _delay_element_11 * _a11 - _delay_element_21 * _a21;
				// do the filtering
				if (isnan(delay_element_0) || isinf(delay_element_0)) {
						// don't allow bad values to propogate via the filter
						delay_element_0 = sample;
				}
				output = delay_element_0 * _b01 + _delay_element_11 * _b11 + _delay_element_21 * _b21;
				
				_delay_element_21 = _delay_element_11;
				_delay_element_11 = delay_element_0;

				// return the value.  Should be no need to check limits
				return output;
		}
}

static float           _cutoff_freq2; 
static float           _a12;
static float           _a22;
static float           _b02;
static float           _b12;
static float           _b22;
static float           _delay_element_12;        // buffered sample -1
static float           _delay_element_22;        // buffered sample -2
void LPF2pSetCutoffFreq_2(float sample_freq, float cutoff_freq)
{
		float fr =0;  
    float ohm =0;
    float c =0;
	
		fr= sample_freq/cutoff_freq;
		ohm=tanf(M_PI_F/fr);
		c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
	
    _cutoff_freq2 = cutoff_freq;
    if (_cutoff_freq2 > 0.0f) 
		{
				_b02 = ohm*ohm/c;
				_b12 = 2.0f*_b02;
				_b22 = _b02;
				_a12 = 2.0f*(ohm*ohm-1.0f)/c;
				_a22 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
		}
}

float LPF2pApply_2(float sample)
{
	
		float delay_element_0 = 0, output=0;
    if (_cutoff_freq2 <= 0.0f) {
        // no filtering
        return sample;
    }
		else
		{
				delay_element_0 = sample - _delay_element_12 * _a12 - _delay_element_22 * _a22;
				// do the filtering
				if (isnan(delay_element_0) || isinf(delay_element_0)) {
						// don't allow bad values to propogate via the filter
						delay_element_0 = sample;
				}
				output = delay_element_0 * _b02 + _delay_element_12 * _b12 + _delay_element_22 * _b22;
				
				_delay_element_22 = _delay_element_12;
				_delay_element_12 = delay_element_0;

				// return the value.  Should be no need to check limits
				return output;
		}
}

static float           _cutoff_freq3; 
static float           _a13;
static float           _a23;
static float           _b03;
static float           _b13;
static float           _b23;
static float           _delay_element_13;        // buffered sample -1
static float           _delay_element_23;        // buffered sample -2
void LPF2pSetCutoffFreq_3(float sample_freq, float cutoff_freq)
{
		float fr =0;  
    float ohm =0;
    float c =0;
	
		fr= sample_freq/cutoff_freq;
		ohm=tanf(M_PI_F/fr);
		c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
	
    _cutoff_freq3 = cutoff_freq;
    if (_cutoff_freq3 > 0.0f) 
		{
				_b03 = ohm*ohm/c;
				_b13 = 2.0f*_b03;
				_b23 = _b03;
				_a13 = 2.0f*(ohm*ohm-1.0f)/c;
				_a23 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
		}
}

float LPF2pApply_3(float sample)
{
	
		float delay_element_0 = 0, output=0;
    if (_cutoff_freq3 <= 0.0f) {
        // no filtering
        return sample;
    }
		else
		{
				delay_element_0 = sample - _delay_element_13 * _a13 - _delay_element_23 * _a23;
				// do the filtering
				if (isnan(delay_element_0) || isinf(delay_element_0)) {
						// don't allow bad values to propogate via the filter
						delay_element_0 = sample;
				}
				output = delay_element_0 * _b03 + _delay_element_13 * _b13 + _delay_element_23 * _b23;
				
				_delay_element_23 = _delay_element_13;
				_delay_element_13 = delay_element_0;

				// return the value.  Should be no need to check limits
				return output;
		}
}
  
 
static float           _cutoff_freq4; 
static float           _a14;
static float           _a24;
static float           _b04;
static float           _b14;
static float           _b24;
static float           _delay_element_14;        // buffered sample -1
static float           _delay_element_24;        // buffered sample -2
void LPF2pSetCutoffFreq_4(float sample_freq, float cutoff_freq)
{
		float fr =0;  
    float ohm =0;
    float c =0;
	
		fr= sample_freq/cutoff_freq;
		ohm=tanf(M_PI_F/fr);
		c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
	
    _cutoff_freq4 = cutoff_freq;
    if (_cutoff_freq4 > 0.0f) 
		{
				_b04 = ohm*ohm/c;
				_b14 = 2.0f*_b04;
				_b24 = _b04;
				_a14 = 2.0f*(ohm*ohm-1.0f)/c;
				_a24 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
		}
}

float LPF2pApply_4(float sample)
{
	
		float delay_element_0 = 0, output=0;
    if (_cutoff_freq4 <= 0.0f) {
        // no filtering
        return sample;
    }
		else
		{
				delay_element_0 = sample - _delay_element_14 * _a14 - _delay_element_24 * _a24;
				// do the filtering
				if (isnan(delay_element_0) || isinf(delay_element_0)) {
						// don't allow bad values to propogate via the filter
						delay_element_0 = sample;
				}
				output = delay_element_0 * _b04 + _delay_element_14 * _b14 + _delay_element_24 * _b24;
				
				_delay_element_24 = _delay_element_14;
				_delay_element_14 = delay_element_0;

				// return the value.  Should be no need to check limits
				return output;
		}
}
 
 
 
static float           _cutoff_freq5; 
static float           _a15;
static float           _a25;
static float           _b05;
static float           _b15;
static float           _b25;
static float           _delay_element_15;        // buffered sample -1
static float           _delay_element_25;        // buffered sample -2
void LPF2pSetCutoffFreq_5(float sample_freq, float cutoff_freq)
{
		float fr =0;  
    float ohm =0;
    float c =0;
	
		fr= sample_freq/cutoff_freq;
		ohm=tanf(M_PI_F/fr);
		c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
	
    _cutoff_freq5 = cutoff_freq;
    if (_cutoff_freq5 > 0.0f) 
		{
				_b05 = ohm*ohm/c;
				_b15 = 2.0f*_b05;
				_b25 = _b05;
				_a15 = 2.0f*(ohm*ohm-1.0f)/c;
				_a25 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
		}
}

float LPF2pApply_5(float sample)
{
	
		float delay_element_0 = 0, output=0;
    if (_cutoff_freq5 <= 0.0f) {
        // no filtering
        return sample;
    }
		else
		{
				delay_element_0 = sample - _delay_element_15 * _a15 - _delay_element_25 * _a25;
				// do the filtering
				if (isnan(delay_element_0) || isinf(delay_element_0)) {
						// don't allow bad values to propogate via the filter
						delay_element_0 = sample;
				}
				output = delay_element_0 * _b05 + _delay_element_15 * _b15 + _delay_element_25 * _b25;
				
				_delay_element_25 = _delay_element_15;
				_delay_element_15 = delay_element_0;

				// return the value.  Should be no need to check limits
				return output;
		}
}
  

 
static float           _cutoff_freq6; 
static float           _a16;
static float           _a26;
static float           _b06;
static float           _b16;
static float           _b26;
static float           _delay_element_16;        // buffered sample -1
static float           _delay_element_26;        // buffered sample -2
void LPF2pSetCutoffFreq_6(float sample_freq, float cutoff_freq)
{
		float fr =0;  
    float ohm =0;
    float c =0;
	
		fr= sample_freq/cutoff_freq;
		ohm=tanf(M_PI_F/fr);
		c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
	
    _cutoff_freq6 = cutoff_freq;
    if (_cutoff_freq6 > 0.0f) 
		{
				_b06 = ohm*ohm/c;
				_b16 = 2.0f*_b06;
				_b26 = _b06;
				_a16 = 2.0f*(ohm*ohm-1.0f)/c;
				_a26 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
		}
}

float LPF2pApply_6(float sample)
{
	
		float delay_element_0 = 0, output=0;
    if (_cutoff_freq6 <= 0.0f) {
        // no filtering
        return sample;
    }
		else
		{
				delay_element_0 = sample - _delay_element_16 * _a16 - _delay_element_26 * _a26;
				// do the filtering
				if (isnan(delay_element_0) || isinf(delay_element_0)) {
						// don't allow bad values to propogate via the filter
						delay_element_0 = sample;
				}
				output = delay_element_0 * _b06 + _delay_element_16 * _b16 + _delay_element_26 * _b26;
				
				_delay_element_26 = _delay_element_16;
				_delay_element_16 = delay_element_0;

				// return the value.  Should be no need to check limits
				return output;
		}
}

static float           _cutoff_freq7; 
static float           _a17;
static float           _a27;
static float           _b07;
static float           _b17;
static float           _b27;
static float           _delay_element_17;        // buffered sample -1
static float           _delay_element_27;        // buffered sample -2
void LPF2pSetCutoffFreq_7(float sample_freq, float cutoff_freq)
{
		float fr =0;  
    float ohm =0;
    float c =0;
	
		fr= sample_freq/cutoff_freq;
		ohm=tanf(M_PI_F/fr);
		c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
	
    _cutoff_freq7 = cutoff_freq;
    if (_cutoff_freq7 > 0.0f) 
		{
				_b07 = ohm*ohm/c;
				_b17 = 2.0f*_b07;
				_b27 = _b07;
				_a17 = 2.0f*(ohm*ohm-1.0f)/c;
				_a27 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
		}
}

float LPF2pApply_7(float sample)
{
	
		float delay_element_0 = 0, output=0;
    if (_cutoff_freq7 <= 0.0f) {
        // no filtering
        return sample;
    }
		else
		{
				delay_element_0 = sample - _delay_element_17 * _a17 - _delay_element_27 * _a27;
				// do the filtering
				if (isnan(delay_element_0) || isinf(delay_element_0)) {
						// don't allow bad values to propogate via the filter
						delay_element_0 = sample;
				}
				output = delay_element_0 * _b07 + _delay_element_17 * _b17 + _delay_element_27 * _b27;
				
				_delay_element_27 = _delay_element_17;
				_delay_element_17 = delay_element_0;

				// return the value.  Should be no need to check limits
				return output;
		}
}