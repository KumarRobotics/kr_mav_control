#include <iostream>
#include <stdexcept>

#include <trajectory_tracker/trajectory_base.h>

TrajSection1D::TrajSection1D(unit n_p_, uint k_r_, decimal_t dt_, std::shared_ptr<BasisBundle> basis_): n_p(n_p_), k_r(k_r_), dt(dt_), basis(basis_) {
	generated = false;
	for(unit i = 0 ; i < n_p ; i++) {
		coeffs_var.push_back( model->addVar(-GRB_INFINITY,GRB_INFINITY,0.0,GRB_CONTINUOUS));
	}	
	// Integrate new variables
	model->update();
}
TrajSection1D::~TrajSection1D() {

}
GRBLinExpr TrajSection1D::getCost() {
	// adds cost based on assumption of using a legendre basis
	GRBQuadExpr objective;
	for(uint i = 1 ; i <= n_p - k_r ; i++) {
		int index = i + k_r;
		double delta = dt/(1.0 + 2.0*(double)i);
		objective += delta * coeff_var[index-1] * coeff_var[index-1];
	}
	return objective;
}
GRBLinExpr TrajSection4D::getCost() {
	GRBQuadExpr objective;
	// add cost from each dimension
	for(auto &sec: secs) {
		objective += sec->getCost();
	}

}

GRBLinExpr TrajSection1D::getContr(decimal_t x, uint derr) {
	GRBLinExpr expr;
	for(uint i = 0 ; i < n_p ; i++)
		expr += basis->getVal(x,dt,i,derr)*coeff_var[i];

}
GRBLinExpr TrajSection4D::getContr(decimal_t x, uint derr, uint dim) {
	return secs[dim]->getContr(x,derr);
}


decimal_t TrajSection1D::evaluate(decimal_t t, uint derr) {
	if(!generated)
		throw std::exception("Trying to evaluate unoptimized trajectory");

	decimal_t rsn = 0;
	for(int i = 0 ; i < n_p ; i ++) {
		rsn += coeffs[i] * basis.getVal(t,dt,i,0);
	}
	return rsn;
}

TrajSection4D::evaluate(decimal_t t, uint derr , Vec4 &out) {
	out == Vec4::Zero();
	for(uint i = 0 ; i < 4 ; i++)
		out(i,1) = secs[i].evaluate(t,derr);
}
TrajSection4D::TrajSection4D(unit n_p, uint k_r, decimal_t dt, std::shared_ptr<BasisBundle> basis){
	for(uint i = 0 ; i < 4 ; i++)
		secs.push_back(new TrajSection1D(n_p,k_r,dt,basis);
}

void TrajSection1D::recoverVars(){
	coeffs.clear();
	for(auto &var: coeff_var)
		coeffs.push_back(var.get(GRB_DoubleAttr_X));
	generated = true;
}
void TrajSection4D::recoverVars(){
	for(auto &sec: secs)
		sec->recoverVars();
}
bool Trajectory::recoverVars() {
	if(model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
		for(auto &sec: individual_sections)
			sec->recoverVars();
		return true;
	}
	else
		return false;
}
bool Trajectory::evaluate(decimal_t t, uint derr, Vec4 &out) {  // returns false when out of time range, but still sets out to endpoint
	if(t < 0) {
		individual_sections.front()->evaluate(0.0,derr,out);
		return false;
	}
	else {
		// find appropriate section
		auto dt_it = dts.begin();
		for(auto &it : individual_sections)
			if(t < *dt_it) {
				it->evaluate(t/(*dt_it),derr,out);
				return true;

			}
			t -= *dt_it;

			++dt_it;
		}
		traj_secs.back()->evaluate(1.0,derr,out);
		return false;
	}
}

Trajectory::Trajectory(GRBModel *model_, const std::vector<Mat4> &waypoints, const std::vector<decimal_t> dts): model(model_) {
	uint n_r = 13; uint k_p = 4; // TODO these should be read of parameter server
	std::shared_ptr<BasisBundle> basis(new BasisBundle(n_r,k_p));

	uint num_secs = waypoints.size() - 1;
	
	if(num_secs <= 0)
		throw std::runtime_error("Not enough waypoints to define trajectory");
	if(num_secs != dts.size())
		throw std::runtime_error("Number of waypoints and list of dt dimension mismatch");

	// create space for sections
	for(uint i = 0; i < num_secs; i++) {
		individual_sections.push_back(new TrajSection4D(n_p,k_r,dts[i],basis));
	}

	
}
void linkSections(const std::vector<Mat4> &waypoints) {
	GRBQuadExpr cost;
	// set start and goal constr

	for(uint i = 0 ; i< 4; i++) {
		for(uint j = 0 ; j< 4; j++) {
			decimal_t constr_start = waypoints.front().at(i,j);
			decimal_t constr_goal  = waypoints.back().at(i,j);
			GRBLinExpr start = individual_sections.front()->getContr(0.0,j,i);		
			GRBLinExpr goal =  individual_sections.back()->getContr(1.0,j,i);
			if(!isnan(constr_start))
				model->addConstr(constr_start == start);
			if(!isnan(constr_goal))
				model->addConstr(constr_goal == goal);
			
		}
	}

	// set right goals
	for(uint i = 0; i < individual_sections.size() - 1) {
		for(uint j = 0 ; j< 4; j++) {
			cost += individual_sections->getCost(j);
			// derrivatives
			for(uint k= 0; k < 4; k++) {
				decimal_t constr = waypoints[i+1].at(j,k);
				GRBLinExpr left = individual_sections[i]->getContr(1.0,k,j);		
				GRBLinExpr right = individual_sections[i+1]->getContr(0.0,k,j);
				model->addConstr(left == right);
				// adds hard coded constraints only is they aren't nan
				if(!isnan(constr))
					model->addConstr(left == constr);		
			}
		}
	}
}

void addMaximumBound(decimal_t bound, uint derr) {
	// use square to approimate circle, TODO have option for arbitray shape
	static decimal_t r2o2 = 0.7071067811865476;
	decimal_t side = bound*r2o2;
	
	for(auto &sec: individual_sections) {
		for(uint dim = 0 ; dim < 4; dim ++) {
			for(double t = 0.0 ; t < 1.0; t+=0.1) { // hard code inforcing contraints at 10 points along sections
				GRBLinExpr expr = sec->getContr(t,derr,dim);
				model->addConstr(expr <=  side);
				model->addConstr(expr >= -side);
			}					
		}
	}



}

int main() {
	std::cout << "run" << std::endl;
	return 0;
};