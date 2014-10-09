// Implementation File
// Look at legendre.h for documentation


template <class T>
Poly<T> PolyCalculus<T>::integrate(const Poly<T> &p) { // integrates polynomial with 0 as constant of integration
	typename Poly<T>::size_type rows = p.size();
	typename std::vector<T> v;
	v.push_back(0.0);
	for(typename Poly<T>::size_type i = 0 ; i < rows; i++) {
		T val = static_cast<T>(i+1);
		v.push_back(p[i]/val);
	}
	Poly<T> result(v.data(),rows);
	return result;

}
template <class T>
Poly<T> PolyCalculus<T>::differentiate(const Poly<T> &p) { // differentiates polynomial
	typename Poly<T>::size_type rows = p.size();
	if(rows == 1)
		return Poly<T>(0.0);
	typename std::vector<T> v;
	for(typename Poly<T>::size_type i = 1 ; i < rows; i++) {
		T val = static_cast<T>(i);
		v.push_back(p[i]*val);
	}
	Poly<T> result(v.data(),rows-2);
	return result;

}

template <class T>
Poly<T> PolyCalculus<T>::shifted_legendre(typename Poly<T>::size_type n) { // shifted legensdre polynomial of order n
	typename std::vector<T> v;
	for(typename Poly<T>::size_type k = 0 ; k <= n; k++) {
		v.push_back(	boost::math::binomial_coefficient<T>(n, k) *
						boost::math::binomial_coefficient<T>(n+k, k) *
						std::pow(-1.0,k+n) );
	}
	Poly<T> result(v.data(),n);
	return result;

}



BasisBundle::BasisBundle(uint n_p_,uint k_r_) : n_p(n_p_), k_r(k_r_) {
	derrivatives.reserve(k_r);
	LegendreBasis base(n_p,k_r);
	// std::cout << "basis " << base << std::endl;
	derrivatives.push_back(base);
	for(int i = 0; i < k_r - 1; i++ ) {
		base.differentiate();
		// std::cout << "basis " << base << std::endl;
		derrivatives.push_back(base);
	}

}
LegendreBasis::~LegendreBasis(){}
LegendreBasis::LegendreBasis(uint n_p_,uint k_r_) : Basis(n_p_,k_r_){
	std::vector<decimal_t> simple;
	simple.push_back(1.0);
	for(uint i = 0; i< k_r ; i++) {
		LPoly poly(simple.data(),simple.size()-1);
		polys.push_back(poly);
		simple.back() = 0.0;
		simple.push_back(1.0);
	}
	for(uint p = 1 ; p <= n_p - k_r; p++) {
		LPoly p_cur = PolyCalculus<decimal_t>::shifted_legendre(p);
		// std::cout << "shifted p " << p_cur << std::endl;
		for(uint i = 0; i< k_r ; i++) {
			p_cur =  PolyCalculus<decimal_t>::integrate(p_cur);
		}
		polys.push_back(p_cur);
		
	}
}
void LegendreBasis::differentiate() {
	for(std::vector<LPoly>::iterator it = polys.begin(); it != polys.end() ; ++it) {
		*it = PolyCalculus<decimal_t>::differentiate(*it);
	}
}
decimal_t LegendreBasis::evaluate(decimal_t x, uint coeff) {
	assert(coeff < polys.size());
	if(x > 1.0 || x < 0.0)
		throw std::out_of_range("Tried to evaluate shifted legensdre basis out of normalized range [0,1]");
	return polys[coeff].evaluate(x);
}
std::ostream& operator<<(std::ostream& os, const LegendreBasis& lb) {
	for(std::vector<LPoly>::const_iterator it = lb.polys.begin(); it != lb.polys.end() ; ++it) {
		os << (*it) << std::endl;
	}
	return os;
}

decimal_t BasisBundle::getVal(decimal_t x, decimal_t dt, uint coeff, uint derr) {
	assert(derr < derrivatives.size());
	decimal_t factor = std::pow(dt, (decimal_t)(k_r - derr));
	return factor*derrivatives[derr].evaluate(x,coeff);
}
BasisBundle::~BasisBundle(){}

Basis::Basis(uint n_p_,uint k_r_) : n_p(n_p_), k_r(k_r_) {}
Basis::~Basis() {}