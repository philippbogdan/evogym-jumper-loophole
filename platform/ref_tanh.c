/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunSoft, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */
/* Match the operation order of glibc's generic fdlibm-based double-precision maths.
 * setup.sh links this file only with REFERENCE_MATH=1 and disables contraction. */
#include <stdint.h>
#include <string.h>
#include <math.h>

#define HIDDEN __attribute__((visibility("hidden")))

static uint32_t hiword(double x) { uint64_t b; memcpy(&b, &x, 8); return (uint32_t)(b >> 32); }
static uint32_t loword(double x) { uint64_t b; memcpy(&b, &x, 8); return (uint32_t)(b & 0xffffffffu); }
static double sethi(double x, uint32_t h) { uint64_t b; memcpy(&b, &x, 8); b = (b & 0xffffffffull) | ((uint64_t)h << 32); memcpy(&x, &b, 8); return x; }

static double ref_expm1(double x) {
	const double huge = 1.0e+300, tiny = 1.0e-300, one = 1.0;
	const double o_threshold = 7.09782712893383973096e+02, ln2_hi = 6.93147180369123816490e-01,
		ln2_lo = 1.90821492927058770002e-10, invln2 = 1.44269504088896338700e+00;
	const double Q1 = -3.33333333333331316428e-02, Q2 = 1.58730158725481460165e-03, Q3 = -7.93650757867487942473e-05,
		Q4 = 4.00821782732936239552e-06, Q5 = -2.01099218183624371326e-07;
	double y, hi, lo, c = 0, t, e, hxs, hfx, r1, h2, h4, R1, R2, R3;
	int32_t k, xsb; uint32_t hx;
	hx = hiword(x);
	xsb = hx & 0x80000000;
	if (xsb == 0) y = x; else y = -x;
	(void)y;
	hx &= 0x7fffffff;
	if (hx >= 0x4043687A) {
		if (hx >= 0x40862E42) {
			if (hx >= 0x7ff00000) {
				uint32_t low = loword(x);
				if (((hx & 0xfffff) | low) != 0) return x + x;
				else return (xsb == 0) ? x : -1.0;
			}
			if (x > o_threshold) return huge * huge;
		}
		if (xsb != 0) return tiny - one;
	}
	if (hx > 0x3fd62e42) {
		if (hx < 0x3FF0A2B2) {
			if (xsb == 0) { hi = x - ln2_hi; lo = ln2_lo; k = 1; }
			else { hi = x + ln2_hi; lo = -ln2_lo; k = -1; }
		} else {
			k = (int32_t)(invln2 * x + ((xsb == 0) ? 0.5 : -0.5));
			t = k;
			hi = x - t * ln2_hi;
			lo = t * ln2_lo;
		}
		x = hi - lo;
		c = (hi - x) - lo;
	} else if (hx < 0x3c900000) {
		t = huge + x;
		return x - (t - (huge + x));
	} else k = 0;
	hfx = 0.5 * x;
	hxs = x * hfx;
	R1 = one + hxs * Q1; h2 = hxs * hxs;
	R2 = Q2 + hxs * Q3; h4 = h2 * h2;
	R3 = Q4 + hxs * Q5;
	r1 = R1 + h2 * R2 + h4 * R3;
	t = 3.0 - r1 * hfx;
	e = hxs * ((r1 - t) / (6.0 - x * t));
	if (k == 0) return x - (x * e - hxs);
	e = (x * (e - c) - c);
	e -= hxs;
	if (k == -1) return 0.5 * (x - e) - 0.5;
	if (k == 1) {
		if (x < -0.25) return -2.0 * (e - (x + 0.5));
		else return one + 2.0 * (x - e);
	}
	if (k <= -2 || k > 56) {
		y = one - (e - x);
		y = sethi(y, hiword(y) + ((uint32_t)k << 20));
		return y - one;
	}
	t = one;
	if (k < 20) {
		t = sethi(t, 0x3ff00000 - (0x200000 >> k));
		y = t - (e - x);
		y = sethi(y, hiword(y) + ((uint32_t)k << 20));
	} else {
		t = sethi(t, ((0x3ff - k) << 20));
		y = x - (e + t);
		y += one;
		y = sethi(y, hiword(y) + ((uint32_t)k << 20));
	}
	return y;
}

/* Bind simulator calls to this implementation without exporting tanh to other modules. */
HIDDEN double tanh(double x) {
	const double one = 1.0, two = 2.0, tiny = 1.0e-300;
	double t, z;
	int32_t jx = (int32_t)hiword(x), ix; uint32_t lx = loword(x);
	ix = jx & 0x7fffffff;
	if (ix >= 0x7ff00000) {
		if (jx >= 0) return one / x + one;
		else return one / x - one;
	}
	if (ix < 0x40360000) {
		if ((ix | lx) == 0) return x;
		if (ix < 0x3c800000) return x * (one + x);
		if (ix >= 0x3ff00000) {
			t = ref_expm1(two * fabs(x));
			z = one - two / (t + two);
		} else {
			t = ref_expm1(-two * fabs(x));
			z = -t / (t + two);
		}
	} else z = one - tiny;
	return (jx >= 0) ? z : -z;
}
