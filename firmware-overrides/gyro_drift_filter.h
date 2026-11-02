/**
 * gyro_drift_filter.h - drift + noise stage for the DGB ICM-45686 firmware.
 *
 * Input/output units are raw gyro counts after axis mapping.
 * At +/-2000 dps, the scale is 16.384 counts per dps.
 */
#ifndef GYRO_DRIFT_FILTER_H_
#define GYRO_DRIFT_FILTER_H_

#include <math.h>
#include <stdint.h>

#ifndef GDF_SAMPLE_HZ
#define GDF_SAMPLE_HZ        980.0f
#endif
#ifndef GDF_COUNTS_PER_DPS
#define GDF_COUNTS_PER_DPS   16.384f
#endif
#ifndef GDF_BLOCK_SAMPLES
#define GDF_BLOCK_SAMPLES    128
#endif
#ifndef GDF_STILL_RANGE
#define GDF_STILL_RANGE      56.0f
#endif
#ifndef GDF_GATE_START
#define GDF_GATE_START       100.0f
#endif
#ifndef GDF_GATE_CONVERGED
#define GDF_GATE_CONVERGED   16.0f
#endif
#ifndef GDF_TAU_START_S
#define GDF_TAU_START_S      1.0f
#endif
#ifndef GDF_TAU_CONVERGED_S
#define GDF_TAU_CONVERGED_S  4.0f
#endif
#ifndef GDF_CONVERGE_BLOCKS
#define GDF_CONVERGE_BLOCKS  8
#endif
#ifndef GDF_MAX_BIAS
#define GDF_MAX_BIAS         100.0f
#endif
#ifndef GDF_DEADZONE
#define GDF_DEADZONE         4.0f
#endif
#ifndef GDF_SPEED_LO
#define GDF_SPEED_LO         50.0f
#endif
#ifndef GDF_SPEED_HI
#define GDF_SPEED_HI         490.0f
#endif
#ifndef GDF_ALPHA_XY_LO
#define GDF_ALPHA_XY_LO      0.15f
#endif
#ifndef GDF_ALPHA_XY_HI
#define GDF_ALPHA_XY_HI      0.40f
#endif
#ifndef GDF_ALPHA_Z_LO
#define GDF_ALPHA_Z_LO       0.08f
#endif
#ifndef GDF_ALPHA_Z_HI
#define GDF_ALPHA_Z_HI       0.20f
#endif

class GyroDriftFilter {
public:
  GyroDriftFilter() { reset(); }

  void reset() {
    for (int i = 0; i < 3; i++) {
      bias_[i] = 0.0f;
      lp_[i] = 0.0f;
    }
    seeded_ = false;
    converged_ = false;
    stillBlocks_ = 0;
    resetWindow();
  }

  bool converged() const { return converged_; }
  float bias(int axis) const { return bias_[axis]; }

  // Input is mapped gyro counts; output is filtered gyro counts.
  void process(float x, float y, float z, float &ox, float &oy, float &oz) {
    const float in[3] = {x, y, z};

    // Collect 128-sample windows and learn bias only during detected stillness.
    for (int i = 0; i < 3; i++) {
      if (in[i] < mn_[i]) mn_[i] = in[i];
      if (in[i] > mx_[i]) mx_[i] = in[i];
      sum_[i] += in[i];
    }
    if (++n_ >= GDF_BLOCK_SAMPLES) {
      endBlock();
      resetWindow();
    }

    // Remove learned bias, then apply a continuous soft deadzone.
    float c[3];
    for (int i = 0; i < 3; i++) {
      const float v = in[i] - bias_[i];
      const float magnitude = fabsf(v) - GDF_DEADZONE;
      c[i] = (magnitude > 0.0f) ? (v > 0.0f ? magnitude : -magnitude) : 0.0f;
    }

    // Adapt smoothing to movement speed: stronger near rest, full response while moving.
    if (!seeded_) {
      for (int i = 0; i < 3; i++) lp_[i] = c[i];
      seeded_ = true;
    } else {
      const float speed = sqrtf(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
      float t = (speed - GDF_SPEED_LO) / (GDF_SPEED_HI - GDF_SPEED_LO);
      if (t < 0.0f) t = 0.0f;
      if (t > 1.0f) t = 1.0f;

      const float alphaXY = GDF_ALPHA_XY_LO + (GDF_ALPHA_XY_HI - GDF_ALPHA_XY_LO) * t;
      const float alphaZ = GDF_ALPHA_Z_LO + (GDF_ALPHA_Z_HI - GDF_ALPHA_Z_LO) * t;
      lp_[0] += alphaXY * (c[0] - lp_[0]);
      lp_[1] += alphaXY * (c[1] - lp_[1]);
      lp_[2] += alphaZ * (c[2] - lp_[2]);
    }

    ox = lp_[0];
    oy = lp_[1];
    oz = lp_[2];
  }

private:
  float bias_[3], lp_[3];
  float mn_[3], mx_[3], sum_[3];
  uint16_t n_, stillBlocks_;
  bool seeded_, converged_;

  void resetWindow() {
    for (int i = 0; i < 3; i++) {
      mn_[i] = 1.0e9f;
      mx_[i] = -1.0e9f;
      sum_[i] = 0.0f;
    }
    n_ = 0;
  }

  void endBlock() {
    const float gate = converged_ ? GDF_GATE_CONVERGED : GDF_GATE_START;
    float mean[3];
    bool still = true;

    for (int i = 0; i < 3; i++) {
      mean[i] = sum_[i] / (float)n_;
      if ((mx_[i] - mn_[i]) > GDF_STILL_RANGE) still = false;
      if (fabsf(mean[i] - bias_[i]) > gate) still = false;
    }

    if (!still) {
      if (!converged_) stillBlocks_ = 0;
      return;
    }

    const float tau = converged_ ? GDF_TAU_CONVERGED_S : GDF_TAU_START_S;
    const float alpha = 1.0f - expf(-(float)GDF_BLOCK_SAMPLES / (tau * GDF_SAMPLE_HZ));
    for (int i = 0; i < 3; i++) {
      bias_[i] += alpha * (mean[i] - bias_[i]);
      if (bias_[i] > GDF_MAX_BIAS) bias_[i] = GDF_MAX_BIAS;
      if (bias_[i] < -GDF_MAX_BIAS) bias_[i] = -GDF_MAX_BIAS;
    }

    if (stillBlocks_ < GDF_CONVERGE_BLOCKS) ++stillBlocks_;
    if (stillBlocks_ >= GDF_CONVERGE_BLOCKS) converged_ = true;
  }
};

#endif
