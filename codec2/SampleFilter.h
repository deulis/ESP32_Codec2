#ifndef SAMPLEFILTER_H_
#define SAMPLEFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 8000 Hz

* 0 Hz - 100 Hz
  gain = 0
  desired attenuation = -20 dB
  actual attenuation = -20.12596954604547 dB

* 240 Hz - 2400 Hz
  gain = 1
  desired ripple = 2 dB
  actual ripple = 1.5173765390636171 dB

* 2500 Hz - 4000 Hz
  gain = 0
  desired attenuation = -20 dB
  actual attenuation = -20.12596954604547 dB

*/

#define SAMPLEFILTER_TAP_NUM 61

typedef struct {
	double history[SAMPLEFILTER_TAP_NUM];
	unsigned int last_index;
} SampleFilter;

void SampleFilter_init(SampleFilter* f);
void SampleFilter_put(SampleFilter* f, double input);
double SampleFilter_get(SampleFilter* f);

#endif