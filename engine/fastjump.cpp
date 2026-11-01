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

// Python bindings for Evolution Gym's own Sim::step and the robot-point
// implementation in fastsim.h. These support replay, rule comparisons, contact
// inspection and energy accounting. Bit identity is reference-platform specific.
#include "fastsim.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <fstream>
#include <sstream>
#include <memory>

namespace py = pybind11;

struct SceneObj { std::string kind, name; int x, y, rows, cols; std::vector<double> grid; int k = 0; std::vector<double> conn; };
struct Scene { int W = 0, H = 0; std::vector<SceneObj> objs; std::vector<int> act; };

static Scene parse_scene(const std::string& text) {
	Scene s; std::istringstream in(text); std::string line;
	while (std::getline(in, line)) {
		std::istringstream is(line); std::string tag; is >> tag;
		if (tag == "init") is >> s.W >> s.H;
		else if (tag == "robot" || tag == "object") {
			SceneObj o; o.kind = tag; is >> o.name >> o.x >> o.y >> o.rows >> o.cols;
			o.grid.resize(o.rows * o.cols); for (auto& v : o.grid) is >> v; s.objs.push_back(o);
		} else if (tag == "conn") {
			SceneObj& o = s.objs.back(); is >> o.k; o.conn.resize(2 * o.k); for (auto& v : o.conn) is >> v;
		} else if (tag == "actuators") { int n; is >> n; s.act.resize(n); for (auto& v : s.act) is >> v; }
	}
	return s;
}

static std::unique_ptr<Sim> build_sim(const Scene& s) {
	std::unique_ptr<Sim> sim(new Sim());
	sim->init(s.W, s.H);
	for (auto& o : s.objs) {
		Matrix<double, Dynamic, Dynamic> g(o.rows, o.cols);
		for (int r = 0; r < o.rows; r++) for (int c = 0; c < o.cols; c++) g(r, c) = o.grid[r * o.cols + c];
		Matrix<double, 2, Dynamic> cm(2, o.k);
		for (int r = 0; r < 2; r++) for (int c = 0; c < o.k; c++) cm(r, c) = o.conn[r * o.k + c];
		bool ok = (o.kind == "robot") ? sim->read_robot_from_array(g, cm, o.name, o.x, o.y)
		                              : sim->read_object_from_array(g, cm, o.name, o.x, o.y);
		if (!ok) throw std::runtime_error("scene load failed");
	}
	return sim;
}

// Match NumPy axis-1 mean on object_pos_at_time: convert units per point, sum in order, then divide.
static inline double com_row(const double* X, int n) {
	double s = 0.0;
	for (int i = 0; i < n; i++) s += (X[i] / 0.1) * 0.1;
	return s / n;
}

struct Clip { double lo = 0.6, hi = 1.6; };

struct Engine {
	Scene scene;
	std::unique_ptr<Sim> sim;  // Upstream simulator supplies the scene for the robot-point implementation.
	FastSim fs;
	double lo = 0.6, hi = 1.6, side = 5.0;
	int n_act;

	Engine(const std::string& scene_text, py::dict params) {
		scene = parse_scene(scene_text);
		sim = build_sim(scene);
		if (params.contains("dt")) fs.P.dt = params["dt"].cast<double>();
		if (params.contains("substeps")) fs.P.substeps = params["substeps"].cast<int>();
		if (params.contains("thresh")) fs.P.thresh = params["thresh"].cast<double>();
		if (params.contains("thresh_rigid")) fs.P.thresh_rigid = params["thresh_rigid"].cast<double>();
		if (params.contains("check_instability")) fs.P.check_instability = params["check_instability"].cast<bool>();
		if (params.contains("contact_per_stage")) fs.P.contact_per_stage = params["contact_per_stage"].cast<bool>();
		if (params.contains("nohide")) fs.P.nohide = params["nohide"].cast<bool>();
		if (params.contains("lo")) lo = params["lo"].cast<double>();
		if (params.contains("hi")) hi = params["hi"].cast<double>();
		if (params.contains("side")) side = params["side"].cast<double>();
		fs.load(*sim, scene.act);
		n_act = scene.act.size();
	}

	inline double clipv(double v) const { v = std::min(std::max(v, lo), hi); if (std::fabs(v) < 1e-8) v = 0; return v; }

	// Replay with the robot-point engine; return score, actions completed, instability and optional y/x histories.
	py::tuple rollout(py::array_t<double, py::array::c_style | py::array::forcecast> A, int T, bool traj) {
		if (A.ndim() != 2 || A.shape(1) != n_act || A.shape(0) < T) throw std::runtime_error("bad action array");
		auto a = A.unchecked<2>();
		fs.reset();
		int NR = fs.NR;
		std::vector<double> act(n_act);
		double cy = com_row(fs.py.data(), NR), cx = com_row(fs.px.data(), NR);
		std::vector<double> ys, xs; if (traj) { ys.push_back(cy); xs.push_back(cx); }
		double R = 0; int t; bool unstable = false;
		for (t = 0; t < T; t++) {
			for (int j = 0; j < n_act; j++) act[j] = clipv(a(t, j));
			fs.set_action(act.data());
			bool done = fs.action_step();
			if (done) { R += -3.0 + (cy - cy) * 10 - std::fabs(cx - cx) * side; unstable = true; t++; break; }
			double ny = com_row(fs.py.data(), NR), nx = com_row(fs.px.data(), NR);
			R += (ny - cy) * 10 - std::fabs(nx - cx) * side;
			cy = ny; cx = nx;
			if (traj) { ys.push_back(cy); xs.push_back(cx); }
		}
		return py::make_tuple(R, t, unstable, py::array(ys.size(), ys.data()), py::array(xs.size(), xs.data()));
	}

	// Record X, Y, VX and VY as [steps + 1, robot points]; goals are [steps, robot edges].
	py::tuple trace(py::array_t<double, py::array::c_style | py::array::forcecast> A, int T) {
		auto a = A.unchecked<2>();
		fs.reset();
		int NR = fs.NR, S = fs.P.substeps;
		std::vector<double> X, Y, VX, VY, G;
		auto rec = [&]() { X.insert(X.end(), fs.px.begin(), fs.px.begin() + NR); Y.insert(Y.end(), fs.py.begin(), fs.py.begin() + NR);
			VX.insert(VX.end(), fs.vx.begin(), fs.vx.end()); VY.insert(VY.end(), fs.vy.begin(), fs.vy.end()); };
		rec();
		std::vector<double> act(n_act);
		int steps = 0; bool done = false;
		for (int t = 0; t < T && !done; t++) {
			for (int j = 0; j < n_act; j++) act[j] = clipv(a(t, j));
			fs.set_action(act.data());
			for (int s = 0; s < S; s++) { G.insert(G.end(), fs.goal.begin(), fs.goal.begin() + fs.NER); done = fs.step(); steps++; rec(); if (done) break; }
		}
		auto mk = [&](std::vector<double>& v, int w) { py::array_t<double> r({(py::ssize_t)(v.size() / w), (py::ssize_t)w}); std::copy(v.begin(), v.end(), r.mutable_data()); return r; };
		return py::make_tuple(mk(X, NR), mk(Y, NR), mk(VX, NR), mk(VY, NR), mk(G, fs.NER));
	}
#ifdef CONTACT_DEBUG
	// Evaluate contacts at the current state; return contact records and net robot forces for the next step.
	py::tuple contacts() {
		fs.dbg_on = true; fs.dbg.clear();
		fs.resolve_collisions(fs.px.data(), fs.py.data(), fs.vtx.data(), fs.vty.data());
		fs.dbg_on = false;
		py::array_t<double> r({(py::ssize_t)fs.dbg.size(), (py::ssize_t)7});
		for (size_t i = 0; i < fs.dbg.size(); i++) for (int j = 0; j < 7; j++) r.mutable_at(i, j) = fs.dbg[i][j];
		int NR = fs.NR;
		return py::make_tuple(r, py::array(NR, fs.cfx.data()), py::array(NR, fs.cfy.data()));
	}
#endif
	py::tuple edges() {
		std::vector<int> a(fs.ea.begin(), fs.ea.begin() + fs.NER), b(fs.eb.begin(), fs.eb.begin() + fs.NER);
		std::vector<double> L(fs.L0.begin(), fs.L0.begin() + fs.NER), K(fs.K.begin(), fs.K.begin() + fs.NER);
		return py::make_tuple(py::array(a.size(), a.data()), py::array(b.size(), b.data()), py::array(L.size(), L.data()), py::array(K.size(), K.data()));
	}


	// For each action, record E = sum(|v|^2/2 + g y) and the signed contributions
	// that decompose the RK4 velocity update and subsequent position projection.
	py::tuple account(py::array_t<double, py::array::c_style | py::array::forcecast> A, int T) {
		auto a = A.unchecked<2>();
		fs.reset();
		int S = fs.P.substeps;
		std::vector<double> act(n_act), Es, terms;
		double acct[FastSim::NT];
		Es.push_back(fs.mech_energy());
		bool done = false; int t;
		for (t = 0; t < T && !done; t++) {
			for (int j = 0; j < n_act; j++) act[j] = clipv(a(t, j));
			fs.set_action(act.data());
			std::fill(acct, acct + FastSim::NT, 0.0);
			fs.acc_on = true;
			for (int s = 0; s < S; s++) { if (fs.step(acct)) { done = true; break; } }
			fs.acc_on = false;
			terms.insert(terms.end(), acct, acct + FastSim::NT);
			Es.push_back(fs.mech_energy());
		}
		py::array_t<double> tr({(py::ssize_t)(terms.size() / FastSim::NT), (py::ssize_t)FastSim::NT});
		std::copy(terms.begin(), terms.end(), tr.mutable_data());
		return py::make_tuple(py::array(Es.size(), Es.data()), tr, t, done);
	}


	// Expose current robot coordinates and integrator velocities as px, py, vx, vy.
	py::tuple state() {
		int NR = fs.NR;
		return py::make_tuple(py::array(NR, fs.px.data()), py::array(NR, fs.py.data()), py::array(NR, fs.vx.data()), py::array(NR, fs.vy.data()));
	}

	// Replay the loaded source Sim::step with Jumper reward; return score, action count, instability and final state.
	py::tuple rollout_source(py::array_t<double, py::array::c_style | py::array::forcecast> A, int T) {
		auto s2 = build_sim(scene);
		auto a = A.unchecked<2>();
		auto p = s2->object_pos_at_time(0, "robot");
		int NR = p.cols();
		double cy;
		std::vector<double> X(NR), Y(NR);
		auto getc = [&](Sim& s, double& cx_, double& cy_) {
			auto q = s.object_pos_at_time(s.get_time(), "robot");
			for (int i = 0; i < NR; i++) { X[i] = q(0, i); Y[i] = q(1, i); }
			cx_ = com_row(X.data(), NR); cy_ = com_row(Y.data(), NR);
		};
		double cx; getc(*s2, cx, cy);
		double R = 0; int t; bool unstable = false;
		for (t = 0; t < T; t++) {
			MatrixXd m(n_act, 2);
			for (int j = 0; j < n_act; j++) { m(j, 0) = scene.act[j]; m(j, 1) = clipv(a(t, j)); }
			s2->set_action("robot", m);
			bool done = s2->step();
			double nx, ny; getc(*s2, nx, ny);
			double r = (ny - cy) * 10 - std::fabs(nx - cx) * side;
			if (done) { r -= 3.0; R += r; unstable = true; t++; break; }
			R += r; cy = ny; cx = nx;
		}
		auto q = s2->pos_at_time(s2->get_time());
		auto v = s2->vel_at_time(s2->get_time());
		std::vector<double> qx(NR), qy(NR), wx(NR), wy(NR);
		for (int i = 0; i < NR; i++) { qx[i] = q(0, i); qy[i] = q(1, i); wx[i] = v(0, i); wy[i] = v(1, i); }
		return py::make_tuple(R, t, unstable, py::array(NR, qx.data()), py::array(NR, qy.data()), py::array(NR, wx.data()), py::array(NR, wy.data()));
	}
};

#ifndef MODNAME
#define MODNAME fastjump
#endif
PYBIND11_MODULE(MODNAME, m) {
	py::class_<Engine>(m, "Engine", py::module_local())
		.def(py::init<const std::string&, py::dict>(), py::arg("scene_text"), py::arg("params") = py::dict())
		.def_readonly("n_act", &Engine::n_act)
		.def("rollout", &Engine::rollout, py::arg("actions"), py::arg("T") = 500, py::arg("traj") = false)
		.def("rollout_source", &Engine::rollout_source)
		.def("state", &Engine::state)
		.def("account", &Engine::account)
		.def("trace", &Engine::trace)
		.def("edges", &Engine::edges)
#ifdef CONTACT_DEBUG
		.def("contacts", &Engine::contacts)
#endif
		.def_property_readonly("n_points", [](Engine& e) { return e.fs.NR; });
}
