/*
MIT License

Copyright (c) 2022 jagdeepsb

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// Evolution Gym's Environment::step and its PhysicsEngine operations, expressed
// as loops over robot points with the source Eigen expression order in binary64.
// ObjectCreator supplies points, edges, cells, surface flags and bounding-box
// trees, retaining the source indices and traversal order. Fixed-ground spring,
// drag and gravity updates, fixed-ground edge constraints and ground-to-ground
// contacts are omitted because they cannot change robot motion.
// Random-body and action comparisons established bit identity for positions,
// velocities and returns on the reference platform; see ../REPRODUCE.md.
#pragma once
#include "Sim.h"
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include <algorithm>

struct FastParams {
	double dt = 0.0001;          // Time interval used by Environment.cpp for RK4 and true-velocity updates.
	int substeps = 30;           // Integration steps per action, matching Sim.cpp.
	double thresh = 0.25;        // General strain threshold, PhysicsEngine.cpp thresh_diff.
	double thresh_rigid = 0.03;  // Rigid strain threshold, PhysicsEngine.cpp thresh_diff_rigid.
	bool check_instability = true;
	bool contact_per_stage = false;  // Re-evaluate contact at each RK4 stage for this rule variant.
	bool nohide = false;             // Close the loophole: a robot point within a ground cell's corners that has no eligible edge is pushed out through the cell's nearest surface edge within half a cell, as patches/close-loophole.patch does.
	bool plain = false;              // With nohide: the top edge, without the containment check, as patches/close-loophole-plain.patch does.
};

struct FBox { int tl, tr, bl, br; int et, eb, el, er; float pts[4]; float eds[4]; int type; int grid_index; };
struct FNode { bool leaf; int a, b, box; double mnx, mny, mxx, mxy; };
struct FObj { std::vector<FBox> boxes; std::vector<FNode> nodes; int root; bool robot; int gw, gh; int nsurf; int nsurf_idx; };

struct FastSim {
	FastParams P;
	int NP = 0, NR = 0, NE = 0, NER = 0;
	std::vector<double> px, py, vx, vy, plx, ply, vtx, vty;
	std::vector<char> fixed;
	std::vector<int> ea, eb;
	std::vector<double> L0, K, Leq, goal;
	std::vector<char> surf;
	std::vector<FObj> objs;
	std::vector<double> cfx, cfy;  // Net contact force components on robot points.
	// Actuator entries follow the supplied action columns.
	std::vector<int> act_type, act_e1, act_e2;
	std::vector<int> rigid_boxes;  // Rigid-cell entries in objs[0].boxes, preserving cell order.
	// Saved starting state used by reset().
	std::vector<double> px0, py0, goal0;
	long steps_done = 0;

	// Diagnostic only, never read by the update: robot points inside the ground that the contact search leaves
	// with no ground force. unpushed_steps counts point-steps; unpushed_depth is the deepest such point, measured
	// from its ground cell's top edge; missed_pairs counts point and cell pairs with no eligible edge.
	long unpushed_steps = 0, missed_pairs = 0;
	double unpushed_depth = 0.0;
	std::vector<int> g_applied; std::vector<double> g_missed;

	// Optional energy and work accumulator fields selected by a non-null acc pointer.
	struct Acc {
		double w_spring = 0, w_drag = 0, w_gravity = 0, w_contact = 0, w_ground_pen = 0;
		double dK_rk = 0;
		double dPE_proj = 0, dK_proj = 0;
	} *acc = nullptr;

	void load(Sim& sim, const std::vector<int>& actuator_grid_indices) {
		Environment& env = sim.environment;
		auto* pos = env.get_pos();
		auto* fx = env.get_fixed();
		auto* edges = env.get_edges();
		auto* objects = env.get_objects();
		NP = pos->cols();
		px.resize(NP); py.resize(NP); fixed.resize(NP);
		for (int i = 0; i < NP; i++) { px[i] = (*pos)(0, i); py[i] = (*pos)(1, i); fixed[i] = (*fx)(0, i); }
		SimObject* robot = objects->at(0);
		if (!robot->is_robot || robot->min_point_index != 0) throw std::runtime_error("robot must be object 0");
		NR = robot->max_point_index + 1;
		for (int i = 0; i < NR; i++) if (fixed[i]) throw std::runtime_error("fixed robot point");
		for (int i = NR; i < NP; i++) if (!fixed[i]) throw std::runtime_error("unfixed non-robot point");
		NE = edges->size();
		ea.resize(NE); eb.resize(NE); L0.resize(NE); K.resize(NE); surf.resize(NE);
		NER = 0;
		for (int e = 0; e < NE; e++) {
			ea[e] = edges->at(e).a_index; eb[e] = edges->at(e).b_index; L0[e] = edges->at(e).init_length_eq;
			K[e] = edges->at(e).spring_const; surf[e] = edges->at(e).isOnSurface;
			bool r = ea[e] < NR || eb[e] < NR;
			if (r) { if (!(ea[e] < NR && eb[e] < NR) || e != NER) throw std::runtime_error("robot edges not a prefix"); NER++; }
		}
		Leq.assign(L0.begin(), L0.end());
		goal.assign(L0.begin(), L0.end());
		objs.clear();
		for (auto* o : *objects) {
			FObj fo; fo.robot = o->is_robot; fo.gw = o->grid_w; fo.gh = o->grid_h;
			fo.nsurf = o->surface_boxels_index.size();
			for (auto& b : o->boxels) {
				FBox fb;
				fb.tl = b.point_top_left_index; fb.tr = b.point_top_right_index; fb.bl = b.point_bot_left_index; fb.br = b.point_bot_right_index;
				fb.et = b.edge_top_index; fb.eb = b.edge_bot_index; fb.el = b.edge_left_index; fb.er = b.edge_right_index;
				for (int k = 0; k < 4; k++) { fb.pts[k] = b.points[k]; fb.eds[k] = b.edges[k]; }
				fb.type = b.cell_type; fb.grid_index = b.grid_index;
				fo.boxes.push_back(fb);
			}
			for (auto& n : o->nodes) {
				FNode fn; fn.leaf = n.is_leaf; fn.a = n.a_index; fn.b = n.b_index; fn.box = n.boxel_index;
				fn.mnx = n.bbox.min_pos.x(); fn.mny = n.bbox.min_pos.y(); fn.mxx = n.bbox.max_pos.x(); fn.mxy = n.bbox.max_pos.y();
				fo.nodes.push_back(fn);
			}
			fo.root = o->tree_root;
			objs.push_back(fo);
		}
		// Ground geometry is fixed, so its bounding boxes need only one update.
		for (size_t i = 1; i < objs.size(); i++) recompute_bbs(objs[i]);
		rigid_boxes.clear();
		for (size_t j = 0; j < objs[0].boxes.size(); j++) if (objs[0].boxes[j].type == CELL_RIGID) rigid_boxes.push_back(j);
		Robot* rb = (Robot*)robot;
		act_type.clear(); act_e1.clear(); act_e2.clear();
		for (int gi : actuator_grid_indices) {
			Boxel* b = rb->get_actuator(gi);
			act_type.push_back(b->cell_type);
			if (b->cell_type == CELL_ACT_H) { act_e1.push_back(b->edge_top_index); act_e2.push_back(b->edge_bot_index); }
			else { act_e1.push_back(b->edge_left_index); act_e2.push_back(b->edge_right_index); }
		}
		vx.assign(NR, 0.0); vy.assign(NR, 0.0);
		auto* vel = env.get_vel();
		for (int i = 0; i < NR; i++) { vx[i] = (*vel)(0, i); vy[i] = (*vel)(1, i); }
		plx.assign(px.begin(), px.begin() + NR); ply.assign(py.begin(), py.begin() + NR);
		vtx = vx; vty = vy;
		cfx.assign(NR, 0.0); cfy.assign(NR, 0.0);
		px0 = px; py0 = py; goal0 = goal;
		actuated.assign(NER, 0);
		for (size_t i = 0; i < act_e1.size(); i++) { actuated[act_e1[i]] = 1; actuated[act_e2[i]] = 1; }
	}

	void reset() {
		px = px0; py = py0; goal = goal0; Leq.assign(L0.begin(), L0.end());
		vx.assign(NR, 0.0); vy.assign(NR, 0.0);
		plx.assign(px.begin(), px.begin() + NR); ply.assign(py.begin(), py.begin() + NR);
		vtx = vx; vty = vy; steps_done = 0;
		unpushed_steps = 0; missed_pairs = 0; unpushed_depth = 0.0;
	}

	// Set target lengths as PhysicsEngine::update_actuator_goals does for Environment::set_robot_action.
	void set_action(const double* a) {
		int n = act_type.size();
		std::vector<int> cnt(NER, -1);
		for (int i = 0; i < n; i++) { goal[act_e1[i]] = 0; goal[act_e2[i]] = 0; cnt[act_e1[i]] = 0; cnt[act_e2[i]] = 0; }
		for (int i = 0; i < n; i++) { goal[act_e1[i]] += a[i]; goal[act_e2[i]] += a[i]; cnt[act_e1[i]] += 1; cnt[act_e2[i]] += 1; }
		for (int e = 0; e < NER; e++) if (cnt[e] >= 0) goal[e] *= L0[e] / cnt[e];
	}

	static inline double cross2(double ax, double ay, double bx, double by) { return ax * by - ay * bx; }

	bool point_in_boxel(const FBox& b, double qx, double qy, const double* X, const double* Y) const {
		double tlx = X[b.tl] - qx, tly = Y[b.tl] - qy;
		double trx = X[b.tr] - qx, try_ = Y[b.tr] - qy;
		double blx = X[b.bl] - qx, bly = Y[b.bl] - qy;
		double brx = X[b.br] - qx, bry = Y[b.br] - qy;
		double a = cross2(tlx, tly, trx, try_);
		double bb = cross2(trx, try_, brx, bry);
		double c = cross2(brx, bry, tlx, tly);
		if (c * a >= 0.0f - 0.0f && c * bb >= 0.0f - 0.0f) return true;
		double d = cross2(tlx, tly, blx, bly);
		double e = cross2(blx, bly, brx, bry);
		return (c * d >= 0.0f - 0.0f && c * e >= 0.0f - 0.0f);
	}

	bool get_intersecting(int e1, int e2, const double* X, const double* Y) const {
		double d1x = X[eb[e1]] - X[ea[e1]], d1y = Y[eb[e1]] - Y[ea[e1]];
		double d2x = X[eb[e2]] - X[ea[e2]], d2y = Y[eb[e2]] - Y[ea[e2]];
		double det = -d1x * d2y + d2x * d1y;
		if (std::abs(det) < 0.0001) return false;
		double xd = X[ea[e2]] - X[ea[e1]];
		double yd = Y[ea[e2]] - Y[ea[e1]];
		double t = (-d2y * xd + d2x * yd) / det;
		double s = (-d1y * xd + d1x * yd) / det;
		const double T = 0.0f;
		return (-T + 0.0f <= t && -T + t <= 1 && -T + 0.0f <= s && -T + s <= 1);
	}

	double dist_point_edge(int p, int e, const double* X, const double* Y) const {
		double bx = X[eb[e]] - X[ea[e]], by = Y[eb[e]] - Y[ea[e]];
		double bl = std::sqrt(bx * bx + by * by);
		if (bl < 0.000001) return 1000.0;
		double sx = X[p] - X[ea[e]], sy = Y[p] - Y[ea[e]];
		double cr = std::abs(bx * sy - sx * by);
		return cr / bl;
	}

	static inline void normalized(double& x, double& y) {
		double z = x * x + y * y;
		if (z > 0.0) { double s = std::sqrt(z); x = x / s; y = y / s; }
	}

	void get_normal(int e, const double* X, const double* Y, double& nx, double& ny) const {
		double sx = X[eb[e]] - X[ea[e]], sy = Y[eb[e]] - Y[ea[e]];
		nx = -sy; ny = sx; normalized(nx, ny);
	}

	// Contact damping and friction use the source vel_true finite-difference velocities.
	const double* CVX; const double* CVY;
	inline double cvx(int i) const { return i < NR ? CVX[i] : 0.0; }
	inline double cvy(int i) const { return i < NR ? CVY[i] : 0.0; }
	inline void addcf(int i, double fx_, double fy_) { if (i < NR) { cfx[i] += fx_; cfy[i] += fy_; } }
	inline void subcf(int i, double fx_, double fy_) { if (i < NR) { cfx[i] -= fx_; cfy[i] -= fy_; } }
	// Record contact-force categories separately, leaving the force-update arithmetic intact.
	// Category indices: ground normal/damping/friction at 0/1/2; self normal/damping/friction at 3/4/5.
	bool acc_on = false;
	std::vector<double> catx[6], caty[6];
	inline void cat_add(int c, int i, double fx_, double fy_) { if (acc_on && i < NR) { catx[c][i] += fx_; caty[c][i] += fy_; } }

	// True containment in a cell, by its corners' bounds. The source's own test, point_in_boxel, also accepts a
	// point lying exactly on the line through a cell's top-left and bottom-right corners, however far away.
	bool within(const FBox& b, double x, double y, const double* X, const double* Y) const {
		double x0 = X[b.tl], x1 = x0, y0 = Y[b.tl], y1 = y0;
		for (int k : {b.tr, b.bl, b.br}) {
			x0 = std::min(x0, X[k]); x1 = std::max(x1, X[k]);
			y0 = std::min(y0, Y[k]); y1 = std::max(y1, Y[k]);
		}
		return x >= x0 && x <= x1 && y >= y0 && y <= y1;
	}

#ifdef CONTACT_DEBUG
	// With CONTACT_DEBUG, log each applied contact as point index, target cell grid index,
	// selected edge, depth, normal-force components and eligible-edge count.
	bool dbg_on = false; std::vector<std::array<double, 7>> dbg;
#endif
	void resolve_point_boxel(int p, const FBox& main, const FBox& ref, const double* X, const double* Y) {
		int me[2]; int nme = 0;
		int mes[4]; int nm = 0;
		if (p == main.tl) { mes[nm++] = main.et; mes[nm++] = main.el; }
		if (p == main.tr) { mes[nm++] = main.et; mes[nm++] = main.er; }
		if (p == main.bl) { mes[nm++] = main.eb; mes[nm++] = main.el; }
		if (p == main.br) { mes[nm++] = main.eb; mes[nm++] = main.er; }
		(void)me; (void)nme;
		int ri[4]; double rs[4]; int nr = 0;
		for (int i = 0; i < 4; i++) {
			int re = (int)ref.eds[i];
			if (!surf[re]) continue;
			bool viable = false;
			for (int j = 0; j < nm; j++) if (get_intersecting(mes[j], re, X, Y)) { viable = true; break; }
			if (viable) { ri[nr] = re; rs[nr] = dist_point_edge(p, re, X, Y); nr++; }
		}
		bool inside = p < NR && (int)ref.eds[0] >= NER && within(ref, X[p], Y[p], X, Y);
		if (inside && surf[(int)ref.eds[0]]) {
			if (nr > 0) g_applied[p]++;
			else { missed_pairs++; double d = dist_point_edge(p, (int)ref.eds[0], X, Y); if (d > g_missed[p]) g_missed[p] = d; }
		}
		if (nr == 0 && P.nohide && p < NR && (int)ref.eds[0] >= NER && P.plain) {
			int re = (int)ref.eds[0];   // Boxel::init orders edges as top, right, bottom, left.
			if (surf[re]) { ri[nr] = re; rs[nr] = dist_point_edge(p, re, X, Y); nr++; }
		}
		if (nr == 0 && P.nohide && !P.plain && inside) {
			double y0 = Y[ref.tl], y1 = y0;
			for (int k : {ref.tr, ref.bl, ref.br}) { y0 = std::min(y0, Y[k]); y1 = std::max(y1, Y[k]); }
			double half = (y1 - y0) / 2;
			for (int i = 0; i < 4; i++) {
				int re = (int)ref.eds[i];
				double d = dist_point_edge(p, re, X, Y);
				if (surf[re] && d < half) { ri[nr] = re; rs[nr] = d; nr++; }
			}
		}
		if (nr == 0) return;
		int im = 0; double md = rs[0];
		for (int i = 0; i < nr; i++) if (rs[i] < md) { md = rs[i]; im = i; }
		int e = ri[im];
		double nx, ny; get_normal(e, X, Y, nx, ny);
		const double cc = 300000.0 * 7, base = 0.005, damp = 140.0;
		double s = base + md;
		double nfx = (nx * cc) * s, nfy = (ny * cc) * s;
		double unx = nx, uny = ny; normalized(unx, uny);
		int a = ea[e], b = eb[e];
		double d1 = unx * cvx(p) + uny * cvy(p);
		double d2 = unx * cvx(a) + uny * cvy(a);
		double d3 = unx * cvx(b) + uny * cvy(b);
		double v1x = ((unx * (-1)) * d1) * damp, v1y = ((uny * (-1)) * d1) * damp;
		double v2x = ((unx * (1)) * d2) * damp, v2y = ((uny * (1)) * d2) * damp;
		double v3x = ((unx * (1)) * d3) * damp, v3y = ((uny * (1)) * d3) * damp;
#ifdef CONTACT_DEBUG
		if (dbg_on) dbg.push_back({(double)p, (double)ref.grid_index, (double)e, md, nfx, nfy, (double)nr});
#endif
		addcf(p, nfx + v1x, nfy + v1y);
		subcf(a, nfx / 2 + v2x / 2, nfy / 2 + v2y / 2);
		subcf(b, nfx / 2 + v3x / 2, nfy / 2 + v3y / 2);
		int cb = (p >= NR || a >= NR || b >= NR) ? 0 : 3;
		if (acc_on) {
			cat_add(cb, p, nfx, nfy); cat_add(cb, a, -nfx / 2, -nfy / 2); cat_add(cb, b, -nfx / 2, -nfy / 2);
			cat_add(cb + 1, p, v1x, v1y); cat_add(cb + 1, a, -v2x / 2, -v2y / 2); cat_add(cb + 1, b, -v3x / 2, -v3y / 2);
		}
		double utx = nx, uty = ny; normalized(utx, uty);
		{ double t = utx; utx = -uty; uty = t; }
		double pvm = utx * cvx(p) + uty * cvy(p);
		double nfm = std::sqrt(nfx * nfx + nfy * nfy);
		if (pvm < 0) { pvm *= -1; utx *= -1; uty *= -1; }
		const double fc = 150.0 * 8, dfc = 0.2;
		double th = std::tanh(pvm / (dfc * nfm));
		double ffx = ((((fc * utx) * (-1)) * dfc) * nfm) * th;
		double ffy = ((((fc * uty) * (-1)) * dfc) * nfm) * th;
		addcf(p, ffx, ffy);
		subcf(a, ffx / 2, ffy / 2);
		subcf(b, ffx / 2, ffy / 2);
		if (acc_on) { int cf_ = (p >= NR || a >= NR || b >= NR) ? 2 : 5; cat_add(cf_, p, ffx, ffy); cat_add(cf_, a, -ffx / 2, -ffy / 2); cat_add(cf_, b, -ffx / 2, -ffy / 2); }
	}

	void resolve_boxel(const FBox& main, const FBox& ref, const double* X, const double* Y) {
		for (int i = 0; i < 4; i++) {
			bool nb = false;
			for (int j = 0; j < 4; j++) if (main.pts[i] == ref.pts[j]) nb = true;
			if (nb) continue;
			int p = (int)main.pts[i];
			if (point_in_boxel(ref, X[p], Y[p], X, Y)) resolve_point_boxel(p, main, ref, X, Y);
		}
	}

	static inline bool bb_in(const FNode& a, const FNode& b) {
		if (a.mnx > b.mxx + 0.0f || 0.0f + a.mxx < b.mnx) return false;
		if (a.mny > b.mxy + 0.0f || 0.0f + a.mxy < b.mny) return false;
		return true;
	}
	static inline double area(const FNode& a) { return (a.mxx - a.mnx) * (a.mxy - a.mny); }

	void traverse(FObj& A, int an, FObj& B, int bn, const double* X, const double* Y) {
		const FNode& a = A.nodes[an]; const FNode& b = B.nodes[bn];
		if (!bb_in(a, b)) return;
		if (a.leaf && b.leaf) {
			resolve_boxel(A.boxes[a.box], B.boxes[b.box], X, Y);
			resolve_boxel(B.boxes[b.box], A.boxes[a.box], X, Y);
			return;
		}
		if (a.leaf) { traverse(A, an, B, b.a, X, Y); traverse(A, an, B, b.b, X, Y); return; }
		if (b.leaf) { traverse(A, a.a, B, bn, X, Y); traverse(A, a.b, B, bn, X, Y); return; }
		if (area(a) < area(b)) { traverse(A, an, B, b.a, X, Y); traverse(A, an, B, b.b, X, Y); }
		else { traverse(A, a.a, B, bn, X, Y); traverse(A, a.b, B, bn, X, Y); }
	}

	int count_self(FObj& A, int an, int bn) {
		const FNode& a = A.nodes[an]; const FNode& b = A.nodes[bn];
		if (!bb_in(a, b)) return 0;
		if (a.leaf && b.leaf) {
			int g1 = A.boxes[a.box].grid_index, g2 = A.boxes[b.box].grid_index;
			if (g1 == g2) return 0;
			int w = A.gw;
			int x1 = g1 % w, y1 = g1 / w, x2 = g2 % w, y2 = g2 / w;
			if (std::abs(x1 - x2) < 1.001 && std::abs(y1 - y2) < 1.001) return 0;
			return 1;
		}
		int c = 0;
		if (a.leaf) { c += count_self(A, an, b.a); c += count_self(A, an, b.b); return c; }
		if (b.leaf) { c += count_self(A, a.a, bn); c += count_self(A, a.b, bn); return c; }
		if (area(a) < area(b)) { c += count_self(A, an, b.a); c += count_self(A, an, b.b); }
		else { c += count_self(A, a.a, bn); c += count_self(A, a.b, bn); }
		return c;
	}

	void recompute_bbs(FObj& o, const double* X = nullptr, const double* Y = nullptr) {
		if (!X) { X = px.data(); Y = py.data(); }
		for (auto& n : o.nodes) {
			if (n.leaf) {
				const FBox& b = o.boxes[n.box];
				int ids[4] = {(int)b.pts[0], (int)b.pts[1], (int)b.pts[2], (int)b.pts[3]};
				double mnx = X[ids[0]], mxx = X[ids[0]], mny = Y[ids[0]], mxy = Y[ids[0]];
				for (int k = 1; k < 4; k++) {
					mnx = std::min(mnx, X[ids[k]]); mxx = std::max(mxx, X[ids[k]]);
					mny = std::min(mny, Y[ids[k]]); mxy = std::max(mxy, Y[ids[k]]);
				}
				n.mnx = mnx; n.mxx = mxx; n.mny = mny; n.mxy = mxy;
			} else {
				const FNode& a = o.nodes[n.a]; const FNode& b = o.nodes[n.b];
				n.mnx = std::min(a.mnx, b.mnx); n.mny = std::min(a.mny, b.mny);
				n.mxx = std::max(a.mxx, b.mxx); n.mxy = std::max(a.mxy, b.mxy);
			}
		}
	}

	// Resolve contacts in the source order, retaining object pairs that can affect the robot.
	void resolve_collisions(const double* X, const double* Y, const double* VX, const double* VY) {
		recompute_bbs(objs[0], X, Y);
		std::fill(cfx.begin(), cfx.end(), 0.0); std::fill(cfy.begin(), cfy.end(), 0.0);
		if (acc_on) for (int c = 0; c < 6; c++) { catx[c].assign(NR, 0.0); caty[c].assign(NR, 0.0); }
		CVX = VX; CVY = VY;
		g_applied.assign(NR, 0); g_missed.assign(NR, 0.0);
		for (size_t i = 0; i < objs.size(); i++)
			for (size_t j = i; j < objs.size(); j++) {
				if (!objs[i].robot && !objs[j].robot) continue;
				traverse(objs[i], objs[i].root, objs[j], objs[j].root, X, Y);
			}
		for (int p = 0; p < NR; p++) if (g_missed[p] > 0.0 && g_applied[p] == 0) {
			unpushed_steps++;
			if (g_missed[p] > unpushed_depth) unpushed_depth = g_missed[p];
		}
	}

	std::vector<double> sfx, sfy;  // Temporary spring-force components.
	// Evaluate the robot acceleration components, following PhysicsEngine::get_pos_vel_slope.
	void slope(const double* X, const double* Y, const double* VX, const double* VY, double* ax, double* ay) {
		for (int i = 0; i < NR; i++) { ax[i] = 0; ay[i] = 0; }
		sfx.resize(NER); sfy.resize(NER);
		for (int e = 0; e < NER; e++) {
			double qx = X[ea[e]] - X[eb[e]], qy = Y[ea[e]] - Y[eb[e]];
			double d = std::sqrt(qx * qx + qy * qy);
			double m = (d - Leq[e]) * K[e] / d;
			sfx[e] = qx * m; sfy[e] = qy * m;
		}
		for (int e = 0; e < NER; e++) { ax[ea[e]] -= sfx[e]; ay[ea[e]] -= sfy[e]; }
		for (int e = 0; e < NER; e++) { ax[eb[e]] += sfx[e]; ay[eb[e]] += sfy[e]; }
		for (int i = 0; i < NR; i++) { ax[i] += VX[i] * (-0.1); ay[i] += VY[i] * (-0.1); }
		const double gm = 1.0 * (-110.0);
		for (int i = 0; i < NR; i++) ay[i] += gm;
		for (int i = 0; i < NR; i++) { ax[i] += cfx[i]; ay[i] += cfy[i]; }
		const double cg = 2000000.0 * 7;
		for (int i = 0; i < NR; i++) {
			bool c = Y[i] < 0;
			double nf = (c ? Y[i] : 0.0) * (-cg);
			ay[i] += nf;
			double ft = std::tanh(VX[i] * nf * 0.2);
			ax[i] += (-10 * 0.2) * (nf * (c ? ft : 0.0));
		}
		// The source division by unit point mass changes no bits.
		if (acc_on) record_stage(X, Y, VX, VY);
	}

	// Store each force category separately at every Runge-Kutta stage.
	// Indices: muscle 0, passive springs 1, drag 2, gravity 3, catx contacts 4 to 9, ground penalty 10.
	static const int NCAT = 11;
	std::vector<char> actuated;
	int stage_idx = 0;
	std::vector<double> sax[4][NCAT], say[4][NCAT];
	void record_stage(const double* X, const double* Y, const double* VX, const double* VY) {
		int s_ = stage_idx;
		for (int c = 0; c < NCAT; c++) { sax[s_][c].assign(NR, 0.0); say[s_][c].assign(NR, 0.0); }
		for (int e = 0; e < NER; e++) {
			int c = actuated[e] ? 0 : 1;
			sax[s_][c][ea[e]] -= sfx[e]; say[s_][c][ea[e]] -= sfy[e];
			sax[s_][c][eb[e]] += sfx[e]; say[s_][c][eb[e]] += sfy[e];
		}
		const double cg = 2000000.0 * 7;
		for (int i = 0; i < NR; i++) {
			sax[s_][2][i] = VX[i] * (-0.1); say[s_][2][i] = VY[i] * (-0.1);
			say[s_][3][i] = -110.0;
			for (int c = 0; c < 6; c++) { sax[s_][4 + c][i] = catx[c][i]; say[s_][4 + c][i] = caty[c][i]; }
			bool cc = Y[i] < 0;
			double nf = (cc ? Y[i] : 0.0) * (-cg);
			double ft = std::tanh(VX[i] * nf * 0.2);
			say[s_][10][i] = nf; sax[s_][10][i] = (-10 * 0.2) * (nf * (cc ? ft : 0.0));
		}
	}

	std::vector<double> k1x, k1y, k1vx, k1vy, k2x, k2y, k2vx, k2vy, k3x, k3y, k3vx, k3vy, k4x, k4y, k4vx, k4vy, tx, ty, tvx, tvy, ax_, ay_;
	void stage_contact(const double* X, const double* Y, const double* VX, const double* VY) {
		// Use stage positions with step-start finite-difference velocities for the contact variant.
		if (!P.contact_per_stage) return;
		(void)VX; (void)VY;
		resolve_collisions(X, Y, vtx.data(), vty.data());
	}
	void rk4(double dt) {
		int N = NR;
		for (auto* v : {&k1x, &k1y, &k1vx, &k1vy, &k2x, &k2y, &k2vx, &k2vy, &k3x, &k3y, &k3vx, &k3vy, &k4x, &k4y, &k4vx, &k4vy, &tx, &ty, &tvx, &tvy, &ax_, &ay_}) v->resize(NP);
		// Copy fixed ground coordinates into the stage buffer without advancing them.
		for (int i = N; i < NP; i++) { tx[i] = px[i]; ty[i] = py[i]; }
		stage_idx = 0;
		auto eval = [&](const double* X, const double* Y, const double* VX, const double* VY, double* kx, double* ky, double* kvx, double* kvy) {
			slope(X, Y, VX, VY, ax_.data(), ay_.data());
			stage_idx++;
			for (int i = 0; i < N; i++) { kx[i] = VX[i] * dt; ky[i] = VY[i] * dt; kvx[i] = ax_[i] * dt; kvy[i] = ay_[i] * dt; }
		};
		eval(px.data(), py.data(), vx.data(), vy.data(), k1x.data(), k1y.data(), k1vx.data(), k1vy.data());
		for (int i = 0; i < N; i++) { tx[i] = px[i] + k1x[i] * 0.5; ty[i] = py[i] + k1y[i] * 0.5; tvx[i] = vx[i] + k1vx[i] * 0.5; tvy[i] = vy[i] + k1vy[i] * 0.5; }
		stage_contact(tx.data(), ty.data(), tvx.data(), tvy.data());
		eval(tx.data(), ty.data(), tvx.data(), tvy.data(), k2x.data(), k2y.data(), k2vx.data(), k2vy.data());
		for (int i = 0; i < N; i++) { tx[i] = px[i] + k2x[i] * 0.5; ty[i] = py[i] + k2y[i] * 0.5; tvx[i] = vx[i] + k2vx[i] * 0.5; tvy[i] = vy[i] + k2vy[i] * 0.5; }
		stage_contact(tx.data(), ty.data(), tvx.data(), tvy.data());
		eval(tx.data(), ty.data(), tvx.data(), tvy.data(), k3x.data(), k3y.data(), k3vx.data(), k3vy.data());
		for (int i = 0; i < N; i++) { tx[i] = px[i] + k3x[i]; ty[i] = py[i] + k3y[i]; tvx[i] = vx[i] + k3vx[i]; tvy[i] = vy[i] + k3vy[i]; }
		stage_contact(tx.data(), ty.data(), tvx.data(), tvy.data());
		eval(tx.data(), ty.data(), tvx.data(), tvy.data(), k4x.data(), k4y.data(), k4vx.data(), k4vy.data());
		for (int i = 0; i < N; i++) {
			px[i] += (((k1x[i] + 2 * k2x[i]) + 2 * k3x[i]) + k4x[i]) / 6;
			py[i] += (((k1y[i] + 2 * k2y[i]) + 2 * k3y[i]) + k4y[i]) / 6;
			vx[i] += (((k1vx[i] + 2 * k2vx[i]) + 2 * k3vx[i]) + k4vx[i]) / 6;
			vy[i] += (((k1vy[i] + 2 * k2vy[i]) + 2 * k3vy[i]) + k4vy[i]) / 6;
		}
	}

	void update_actuators() {
		for (int e = 0; e < NER; e++) {
			double qx = px[ea[e]] - px[eb[e]], qy = py[ea[e]] - py[eb[e]];
			double d = std::sqrt(qx * qx + qy * qy);
			Leq[e] = d + (goal[e] - d) * 0.006;
		}
	}

	std::vector<double> ccx, ccy;
	void resolve_edge_constraints() {
		ccx.resize(NER); ccy.resize(NER);
		const double hi = 1 + P.thresh, lo = 1 - P.thresh;
		for (int e = 0; e < NER; e++) {
			double qx = px[ea[e]] - px[eb[e]], qy = py[ea[e]] - py[eb[e]];
			double d = std::sqrt(qx * qx + qy * qy);
			double mo = (d - L0[e] * hi) / (d * 2);
			double mu = (d - L0[e] * lo) / (d * 2);
			double ox = qx * mo, oy = qy * mo, ux = qx * mu, uy = qy * mu;
			double st = d / L0[e];
			if (!(st > hi)) { ox = 0.0f; oy = 0.0f; }
			if (!(st < lo)) { ux = 0.0f; uy = 0.0f; }
			ccx[e] = (ox + ux) * 0.8; ccy[e] = (oy + uy) * 0.8;
		}
		for (int e = 0; e < NER; e++) { px[ea[e]] -= ccx[e]; py[ea[e]] -= ccy[e]; }
		for (int e = 0; e < NER; e++) { px[eb[e]] += ccx[e]; py[eb[e]] += ccy[e]; }
		const double rhi = 1 + P.thresh_rigid, rlo = 1 - P.thresh_rigid;
		for (int bi : rigid_boxes) {
			const FBox& b = objs[0].boxes[bi];
			for (int k = 0; k < 4; k++) {
				int e = (int)b.eds[k];
				double qx = px[ea[e]] - px[eb[e]], qy = py[ea[e]] - py[eb[e]];
				double d = std::sqrt(qx * qx + qy * qy);
				double mo = (d - L0[e] * rhi) / (d * 2);
				double mu = (d - L0[e] * rlo) / (d * 2);
				double ox = qx * mo, oy = qy * mo, ux = qx * mu, uy = qy * mu;
				double st = d / L0[e];
				if (st < rhi) { ox = 0; oy = 0; }
				if (st > rlo) { ux = 0; uy = 0; }
				px[ea[e]] -= (ox + ux) * 0.5; py[ea[e]] -= (oy + uy) * 0.5;
				px[eb[e]] += (ox + ux) * 0.5; py[eb[e]] += (oy + uy) * 0.5;
			}
		}
	}

	// Advance one Environment::step equivalent; true reports an instability.
	std::vector<double> sx0, sy0;
	// Account columns are named by NAMES and SPLIT in ../scripts/energy.py; the final slot is reserved.
	static const int NT = 18;
	std::vector<double> vx_old, vy_old, py_old, dcur;
	bool step(double* acct = nullptr) {
		resolve_collisions(px.data(), py.data(), vtx.data(), vty.data());
		update_actuators();
		if (acct) {
			for (int e = 0; e < NER; e++) {
				double qx = px[ea[e]] - px[eb[e]], qy = py[ea[e]] - py[eb[e]];
				double d = std::sqrt(qx * qx + qy * qy);
				double off = d - Leq[e];
				acct[actuated[e] ? 13 : 14] += 0.5 * K[e] * off * off;   // Spring energy immediately after resetting its equilibrium length.
			}
			vx_old = vx; vy_old = vy; py_old.assign(py.begin(), py.begin() + NR);
		}
		if (P.contact_per_stage) { sx0 = px; sy0 = py; }
		rk4(P.dt);
		// Restore step-start robot bounding boxes for the later instability test.
		if (P.contact_per_stage) recompute_bbs(objs[0], sx0.data(), sy0.data());
		if (acct) {
			for (int e = 0; e < NER; e++) {
				double qx = px[ea[e]] - px[eb[e]], qy = py[ea[e]] - py[eb[e]];
				double d = std::sqrt(qx * qx + qy * qy);
				double off = d - Leq[e];
				acct[actuated[e] ? 15 : 16] -= 0.5 * K[e] * off * off;   // Negated residual spring energy before projection and the following reset.
			}
			const double dt = P.dt;
			for (int c = 0; c < NCAT; c++) {
				double W = 0;
				for (int i = 0; i < NR; i++) {
					double dvx = (((sax[0][c][i] * dt + 2 * (sax[1][c][i] * dt)) + 2 * (sax[2][c][i] * dt)) + sax[3][c][i] * dt) / 6;
					double dvy = (((say[0][c][i] * dt + 2 * (say[1][c][i] * dt)) + 2 * (say[2][c][i] * dt)) + say[3][c][i] * dt) / 6;
					double mvx = vx_old[i] + 0.5 * (vx[i] - vx_old[i]), mvy = vy_old[i] + 0.5 * (vy[i] - vy_old[i]);
					W += mvx * dvx + mvy * dvy;
				}
				acct[c] += W;
			}
			double dpe = 0; for (int i = 0; i < NR; i++) dpe += 110.0 * (py[i] - py_old[i]);
			acct[11] += dpe;
			py_old.assign(py.begin(), py.begin() + NR);
		}
		resolve_edge_constraints();
		if (acct) {
			double dpe = 0; for (int i = 0; i < NR; i++) dpe += 110.0 * (py[i] - py_old[i]);
			acct[12] += dpe;
		}
		for (int i = 0; i < NR; i++) {
			vtx[i] = (px[i] - plx[i]) / P.dt; vty[i] = (py[i] - ply[i]) / P.dt;
			plx[i] = px[i]; ply[i] = py[i];
		}
		steps_done++;
		if (!P.check_instability) return false;
		int c = count_self(objs[0], objs[0].root, objs[0].root);
		return c * 1.0 > objs[0].nsurf;
	}

	double mech_energy() const {
		double E = 0;
		for (int i = 0; i < NR; i++) E += 0.5 * (vx[i] * vx[i] + vy[i] * vy[i]) + 110.0 * py[i];
		return E;
	}

	// Advance one action using the integration-step count from Sim::step.
	bool action_step() {
		for (int i = 0; i < P.substeps; i++) if (step()) return true;
		return false;
	}
};
