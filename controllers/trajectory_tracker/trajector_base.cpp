#include <iostream>

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


int main() {
	std::cout << "run" << std::endl;
	return 0;
};