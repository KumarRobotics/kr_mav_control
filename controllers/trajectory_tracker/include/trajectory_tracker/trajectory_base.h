#ifndef TRAJECTORY_BASE_H
#define TRAJECTORY_BASE_H

#include <trajectory_tracker/legendre.h>

#include <gurobi_c++.h>
#include <memory>

enum{TRAJ_X = 0, TRAJ_Y = 1, TRAJ_Z = 2, TRAJ_PSI = 3};

class TrajSection1D {
public:
	TrajSection1D(GRBModel *model_, uint n_p_, uint k_r_, decimal_t dt_, std::shared_ptr<BasisBundle> basis);
	~TrajSection1D();

	decimal_t evaluate(decimal_t t, uint derr);

	// gurobi contr functions
	void getContr(decimal_t x, uint derr, GRBLinExpr &expr);
	void getCost(GRBQuadExpr &cost);

	// unpacks optimized coefficients
	void recoverVars();
private:
	uint n_p; // dimension of underlying basis representation
	uint k_r; // order being minimized i.e. 4 = snap
	bool generated; // whether or not trajectory has been generated
	decimal_t dt; // durration of segment

	std::shared_ptr<BasisBundle> basis; // list of bases
	
	// gurobi model
	GRBModel *model;
	std::vector<GRBVar> coeffs_var;
	std::vector<decimal_t> coeffs;

};

// Trajectory section in 4D

class TrajSection4D {
public:
	TrajSection4D(GRBModel *model, uint n_p_, uint k_r_, decimal_t dt_, std::shared_ptr<BasisBundle> basis_);
	~TrajSection4D();

	void evaluate(decimal_t t, uint derr , Vec4 &out);

	void getContr(decimal_t x, uint derr, uint dim, GRBLinExpr &expr); // dim 0,1,2,3 for x,y,z,psi
	void getCost(GRBQuadExpr &cost);

	// unpacks optimized coefficients
	void recoverVars();
private:
	std::vector<std::shared_ptr<TrajSection1D> > secs;

};


class Trajectory {
public:
	Trajectory(GRBModel *model_, const Mat4Vec &waypoints, const std::vector<decimal_t> dts_); // way points and times each column represents a time derrivate: column 0 is position, column 1 is velocity, etc. 
	Trajectory(const Trajectory& traj) = delete; // forbid copying because dependent on gurobi model pointer
	~Trajectory();

	void addMaximumBound(decimal_t bound, uint derr);// ex (3.5,1) sets maximum velocities to 3.5
	bool recoverVars();
	bool evaluate(decimal_t t, uint derr, Vec4 &out); // returns false when out of time range, but returns enpoints
private:
	std::vector<std::shared_ptr<TrajSection4D> > individual_sections;
	void linkSections(const Mat4Vec &waypoints); // links endpoints of trajectories

	GRBModel *model;

	std::vector<decimal_t> dts;
};

#endif