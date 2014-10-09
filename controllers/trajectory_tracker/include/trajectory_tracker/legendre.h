#include <trajectory_tracker/trajectory_types.h>

#include <boost/math/special_functions/binomial.hpp>
#include <boost/math/tools/polynomial.hpp>
#include <cmath>
#include <cassert>
#include <memory>


#ifndef LEGENDRE_H
#define LEGENDRE_H

// Basis virtual class //////////////////////////////////////////////////////////////////////////
class Basis
{
public:
	Basis(uint n_p_,uint k_r_); // dimension of basis, and degree minimizing
	~Basis();
	virtual void differentiate() = 0;
	virtual decimal_t evaluate(decimal_t x, uint coeff) = 0;
protected:
	uint n_p,k_r;
};



// Poly Calculus Functions //////////////////////////////////////////////////////////////////////////

template <class T>
using Poly = typename boost::math::tools::polynomial<T>;

template <class T>
class PolyCalculus {
public:
	static Poly<T> integrate(const Poly<T> &p);  // indefinate integral of polynomial with constant 0
	static Poly<T> differentiate(const Poly<T> &p); // differentiates polynomial

	static Poly<T> shifted_legendre(typename  Poly<T>::size_type n); // shifted legensdre polynomial of order n

};

// Legendre Basis ////////////////////////////////////////////////////////////////////////////////////
typedef Poly<decimal_t> LPoly;
class LegendreBasis : Basis
{
public:
	LegendreBasis(uint n_p_,uint k_r_);
	~LegendreBasis();
	virtual void differentiate();
	virtual decimal_t evaluate(decimal_t x, uint coeff);
	friend std::ostream& operator<<(std::ostream& os, const LegendreBasis& lb);
private:
	std::vector<LPoly> polys; // basis polynomials
};

class BasisBundle // bundles the basis with its derrivatives
{
public:
	BasisBundle(uint n_p_ , uint k_r_);
	~BasisBundle();
	decimal_t getVal(decimal_t x, decimal_t dt, uint coeff, uint derr); // returns value of basis at value x, with time dt, basis function coeff, and derrivative derr
private:
	uint n_p, k_r;
	std::vector<LegendreBasis> derrivatives;

};

#include <trajectory_tracker/legendre.hpp>

#endif