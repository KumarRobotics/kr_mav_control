#ifndef TRAJECTORY_BASE_H
#define TRAJECTORY_BASE_H

#include <gurobi_c++.h>
#include <memory>

enum{TRAJ_X = 0, TRAJ_Y = 1, TRAJ_Z = 2, TRAJ_PSI = 3};

class TrajSection1D {
public:
	TrajSection(unit n_p_, uint k_r_, decimal_t dt_, std::shared_ptr<BasisBundle> basis);
	~TrajSection();

	decimal_t evaluate(decimal_t t, uint derr);
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

	// gurobi contr functions
	GRBLinExpr getContr(decimal_t x, uint derr, uint dim); // dim 0,1,2,3 for x,y,z,psi
	GRBLinExpr getCost(uint dim);
};

class TrajSection4D {
public:
	TrajSection4D();
	~TrajSection4D();

	evaluate(decimal_t t, uint derr , Vec4 &out);

	GRBLinExpr getContr(decimal_t x, uint derr, uint dim); // dim 0,1,2,3 for x,y,z,psi
	GRBLinExpr getCost(uint dim);

private:
	std::vector<std::shared_ptr<TrajSection4D> > secs;

};


class Trajectory {
public:
	Trajectory();
	Trajectory(const Trajectory& traj) = delete;
	~Trajectory();
private:
	std::vector<shared_ptr<TrajSection4D> > individual_sections;

};

#endif